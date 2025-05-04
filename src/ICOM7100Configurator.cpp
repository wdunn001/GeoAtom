#include "ICOM7100Configurator.h"
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <set>
#include <FS.h>
#include <SPIFFS.h>

// Add externs for smart heading/speed and filtered lat/lng
extern float getSmartHeading();
extern float getSmartSpeed();
extern float getLatitude();
extern float getLongitude();

// Constructor implementation
ICOM7100Configurator::ICOM7100Configurator(HardwareSerial& serial) 
    : radio(serial), lastCommandTime(0){
}

// Helper function to send command with checksum
void ICOM7100Configurator::sendCommand(const String& cmd) {
    if (millis() - lastCommandTime < COMMAND_DELAY) {
        // Not enough time has passed, so skip sending this command
        return;
    }
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 0; i < cmd.length(); i++) {
        checksum ^= cmd[i];
    }
    // Send command with checksum
    radio.print(cmd);
    if (checksum < 16) radio.print("0");
    radio.print(checksum, HEX);
    radio.print("\r\n");
    

    
    lastCommandTime = millis();
}

// NMEA Forwarding
void ICOM7100Configurator::forwardNMEAToRadio(TinyGPSPlus& gps, int altitudeCorrection) {
    static unsigned long lastSendTime = 0;
    static unsigned long lastGSVTime = 0;
    static String nmea_buffer = ""; // Buffer to hold incomplete NMEA sentences
    static bool gsaReceived = false;
    static bool gsvReceived = false;
    static unsigned long lastRealMessageTime = 0;
    
    const unsigned long FORWARD_INTERVAL = 500; // 2Hz
    const unsigned long GSV_INTERVAL = 1000; // Generate GSV message every second if not received
    const unsigned long MSG_TIMEOUT = 3000; // Timeout for receiving real messages

    unsigned long now = millis();
    
    // Process GPS data from any connected receiver and forward directly
    while (Serial1.available()) {
        char c = Serial1.read();
        
        // Add character to buffer
        if (c == '$') {
            // New sentence starting, clear buffer
            nmea_buffer = "$";
        } else if (c == '\r' || c == '\n') {
            // End of sentence, process if not empty
            if (nmea_buffer.length() > 5) {
                // Check if it's a valid NMEA sentence
                if (nmea_buffer.indexOf('*') > 0) {
                    // Forward to radio
                    radio.println(nmea_buffer);
                    
                    // Log GSV and GSA messages
                    if (nmea_buffer.startsWith("$GPGSV") || nmea_buffer.startsWith("$GLGSV")) {
                        gsvReceived = true;
                        logMessage("Forwarded real GSV: " + nmea_buffer);
                    } else if (nmea_buffer.startsWith("$GPGSA") || nmea_buffer.startsWith("$GLGSA")) {
                        gsaReceived = true;
                        logMessage("Forwarded real GSA: " + nmea_buffer);
                    } else if (nmea_buffer.startsWith("$GPGGA")) {
                        // Extract real satellite count from GGA for our own reference
                        int satIndex = nmea_buffer.indexOf(',', 7);
                        if (satIndex > 0) {
                            int nextComma = nmea_buffer.indexOf(',', satIndex + 1);
                            if (nextComma > 0) {
                                String realSatStr = nmea_buffer.substring(satIndex + 1, nextComma);
                                if (realSatStr.length() > 0) {
                                    logMessage("Real satellite count from GGA: " + realSatStr);
                                }
                            }
                        }
                    }
   
                    
                    // Update timestamp for last real message
                    lastRealMessageTime = now;
                }
                
                // Clear buffer for next sentence
                nmea_buffer = "";
            }
        } else {
            // Add character to buffer
            nmea_buffer += c;
        }
    }
    
    // Define a consistent satellite count to use across all messages
    int satCount = gps.satellites.isValid() ? gps.satellites.value() : 8;
    if (satCount < 4) satCount = 4; // Minimum for display
    satCount = min(satCount, 12);   // Maximum realistic value
    
    // If we haven't received real GSV/GSA messages for a while, generate them
    // This ensures the radio always has satellite data even if the GPS module
    // temporarily doesn't provide it
    bool useBackupMessages = (now - lastRealMessageTime > MSG_TIMEOUT) || 
                            (!gsvReceived && !gsaReceived);
    
    if (useBackupMessages && (now - lastGSVTime >= GSV_INTERVAL)) {
        logMessage("No real GPS messages received recently. Using backup data with sat count: " + String(satCount));
        
        // Generate GSV and GSA messages with consistent satellite count
        generateBackupGSVMessages(gps, satCount);
        generateBackupGSAMessage(gps, satCount);
        
        lastGSVTime = now;
        
        // Reset flags to allow trying for real data next time
        gsvReceived = false;
        gsaReceived = false;
    }
    
    // Send GGA and RMC messages at regular intervals if not receiving them from GPS
    if (now - lastSendTime >= FORWARD_INTERVAL) {
        bool hasValidData = gps.location.isValid() && gps.time.isValid();

        // Generate GGA message (Global Positioning System Fix Data)
        String ggaMessage = "$GPGGA,";
        
        // Time field
        if (gps.time.isValid()) {
            // Format time as HHMMSS.SS
            String hour = String(gps.time.hour());
            if (gps.time.hour() < 10) hour = "0" + hour;
            String minute = String(gps.time.minute());
            if (gps.time.minute() < 10) minute = "0" + minute;
            String second = String(gps.time.second());
            if (gps.time.second() < 10) second = "0" + second;
            ggaMessage += hour + minute + second + ".00,";
        } else {
            ggaMessage += ","; // Empty field for unknown time
        }

        // Position data
        if (gps.location.isValid()) {
            // Format latitude as DDMM.MMMM
            float lat = abs(getLatitude());
            int latDeg = (int)lat;
            float latMin = (lat - latDeg) * 60.0;
            String latDegStr = String(latDeg);
            if (latDeg < 10) latDegStr = "0" + latDegStr;
            
            char latMinStr[10];
            sprintf(latMinStr, "%07.4f", latMin); // Ensure 4 decimal places with leading zeros
            ggaMessage += latDegStr + latMinStr + "," + (getLatitude() < 0 ? "S," : "N,");

            // Format longitude as DDDMM.MMMM
            float lng = abs(getLongitude());
            int lngDeg = (int)lng;
            float lngMin = (lng - lngDeg) * 60.0;
            String lngDegStr = String(lngDeg);
            if (lngDeg < 100) lngDegStr = "0" + lngDegStr;
            if (lngDeg < 10) lngDegStr = "0" + lngDegStr;
            
            char lngMinStr[10];
            sprintf(lngMinStr, "%07.4f", lngMin); // Ensure 4 decimal places with leading zeros
            ggaMessage += lngDegStr + lngMinStr + "," + (getLongitude() < 0 ? "W," : "E,");

            // Quality indicator and satellite count
            ggaMessage += "1,"; // Fix quality: 1 = GPS fix
            
            // Number of satellites - CRITICAL for satellite display
            // Use the same satCount we're using in GSV/GSA messages for consistency
            // Ensure the satellite count has 2 digits with leading zero if needed
            String satString = String(satCount);
            if (satCount < 10) satString = "0" + satString;
            ggaMessage += satString + ",";
            
            // HDOP and altitude - CRITICAL for altitude display
            if (gps.hdop.isValid()) {
                char hdopStr[10];
                sprintf(hdopStr, "%.1f", gps.hdop.hdop());
                ggaMessage += String(hdopStr) + ",";
            } else {
                ggaMessage += "1.5,"; // Reasonable HDOP value
            }
            
            // Altitude - must be properly formatted for ICOM display
            if (gps.altitude.isValid()) {
                char altStr[10];
                sprintf(altStr, "%.1f", gps.altitude.meters() + altitudeCorrection);
                ggaMessage += String(altStr) + ",M,";
            } else {
                ggaMessage += "0.0,M,"; // Use zero instead of empty for altitude
            }
            
            // Height of geoid above WGS84
            ggaMessage += "0.0,M,";
            
            // Time since last DGPS update and DGPS station ID
            ggaMessage += ",,";
        } else {
            // If location is invalid, use empty fields for coordinates
            ggaMessage += ",,,,"; // Empty lat/lon fields
            ggaMessage += "0,"; // Fix quality: 0 = Invalid
            
            // Satellite count - use the same value as in GSV/GSA
            String satString = String(satCount);
            if (satCount < 10) satString = "0" + satString;
            ggaMessage += satString + ",";
            
            // HDOP, altitude, etc.
            ggaMessage += "1.5,0.0,M,0.0,M,,";
        }

        // Calculate checksum
        uint8_t checksum = 0;
        for (size_t i = 1; i < ggaMessage.length(); i++) {
            checksum ^= ggaMessage[i];
        }
        
        ggaMessage += "*";
        if (checksum < 16) ggaMessage += "0";
        ggaMessage += String(checksum, HEX);
        ggaMessage += "\r\n";
        
        // Send GGA message
        radio.print(ggaMessage);
        ggaMessagesSent++;
        
        // Track message quality
        if (!hasValidData) {
            nullMessages++;
        } else {
            convertedMessages++;
        }

        // Send RMC message (Recommended Minimum Navigation Information)
        String rmcMessage = "$GPRMC,";
        
        // Time field
        if (gps.time.isValid()) {
            // Format time as HHMMSS.SS
            String hour = String(gps.time.hour());
            if (gps.time.hour() < 10) hour = "0" + hour;
            String minute = String(gps.time.minute());
            if (gps.time.minute() < 10) minute = "0" + minute;
            String second = String(gps.time.second());
            if (gps.time.second() < 10) second = "0" + second;
            rmcMessage += hour + minute + second + ".00,";
        } else {
            rmcMessage += ","; // Empty time field
        }

        // Status and position fields
        if (gps.location.isValid()) {
            rmcMessage += "A,"; // Status (A = valid)
            
            // Format latitude as DDMM.MMMM
            float lat = abs(getLatitude());
            int latDeg = (int)lat;
            float latMin = (lat - latDeg) * 60.0;
            String latDegStr = String(latDeg);
            if (latDeg < 10) latDegStr = "0" + latDegStr;
            
            char latMinStr[10];
            sprintf(latMinStr, "%07.4f", latMin); // Ensure 4 decimal places with leading zeros
            rmcMessage += latDegStr + latMinStr + "," + (getLatitude() < 0 ? "S," : "N,");

            // Format longitude as DDDMM.MMMM
            float lng = abs(getLongitude());
            int lngDeg = (int)lng;
            float lngMin = (lng - lngDeg) * 60.0;
            String lngDegStr = String(lngDeg);
            if (lngDeg < 100) lngDegStr = "0" + lngDegStr;
            if (lngDeg < 10) lngDegStr = "0" + lngDegStr;
            
            char lngMinStr[10];
            sprintf(lngMinStr, "%07.4f", lngMin); // Ensure 4 decimal places with leading zeros
            rmcMessage += lngDegStr + lngMinStr + "," + (getLongitude() < 0 ? "W," : "E,");

            // Speed
            float smartSpeed = getSmartSpeed();
            rmcMessage += String(smartSpeed * 0.539957, 1) + ","; // Convert km/h to knots
            
            // Course
            float smartHeading = getSmartHeading();
            if (smartHeading >= 0) {
                rmcMessage += String(smartHeading, 1) + ",";
            } else {
                rmcMessage += "0.0,";
            }
            
            // Date
            if (gps.date.isValid()) {
                // Format date as DDMMYY
                String day = String(gps.date.day());
                if (gps.date.day() < 10) day = "0" + day;
                String month = String(gps.date.month());
                if (gps.date.month() < 10) month = "0" + month;
                String year = String(gps.date.year() % 100);
                if ((gps.date.year() % 100) < 10) year = "0" + year;
                rmcMessage += day + month + year + ",";
            } else {
                // Use current date if unavailable
                rmcMessage += "010123,"; // January 1, 2023 as fallback
            }
            
            // Magnetic variation (typically not available)
            rmcMessage += ",";
            
            // Mode indicator (added in NMEA 2.3)
            rmcMessage += "A";
        } else {
            rmcMessage += "V,"; // V = Navigation receiver warning
            
            // Empty position fields
            rmcMessage += ",,,,"; // Lat/Lon empty
            
            // Speed and course - zero values
            rmcMessage += "0.0,0.0,";
            
            // Date - use fallback
            rmcMessage += "010123,";
            
            // Magnetic variation
            rmcMessage += ",";
            
            // Mode indicator
            rmcMessage += "N"; // N = Data not valid
        }

        // Calculate checksum
        checksum = 0;
        for (size_t i = 1; i < rmcMessage.length(); i++) {
            checksum ^= rmcMessage[i];
        }
        
        rmcMessage += "*";
        if (checksum < 16) rmcMessage += "0";
        rmcMessage += String(checksum, HEX);
        rmcMessage += "\r\n";
        
        // Send RMC message
        radio.print(rmcMessage);
        rmcMessagesSent++;
        
        // Track message quality for RMC
        if (!hasValidData) {
            nullMessages++;
        } else {
            convertedMessages++;
        }
        
        lastSendTime = now;
    }

    // Check for non-standard messages from the radio
    while (radio.available()) {
        String line = radio.readStringUntil('\n');
        line.trim();
        if (line.startsWith("$GMEA")) {
            logMessage("Received non-standard GMEA: " + line);
            messageErrors++;
        }
    }

    // Report statistics every 10 seconds
    reportGPSStats();
}

struct YumaSatInfo {
    int prn;
    int health;
    double ecc;
    double toa;
    double inc;
    double rasc_rate;
    double sqrtA;
    double rasc;
    double argp;
    double mean_anom;
    double af0;
    double af1;
    int week;
};

static std::vector<YumaSatInfo> yumaSats;
static bool yumaLoaded = false;

// Helper: Parse YUMA file for all healthy satellites (SPIFFS version)
static void loadYumaAlmanacFull() {
    if (yumaLoaded) return;
    yumaLoaded = true;
    yumaSats.clear();
    if (!SPIFFS.begin(true)) {
        logMessage("ERROR: SPIFFS mount failed");
        return;
    }
    File file = SPIFFS.open("/gps_almanac/current_yuma.txt", "r");
    if (!file) {
        logMessage("ERROR: Could not open /gps_almanac/current_yuma.txt");
        return;
    }
    String line;
    YumaSatInfo sat = {};
    while (file.available()) {
        line = file.readStringUntil('\n');
        line.trim();
        if (line.indexOf("almanac for PRN-") != -1) {
            int pos = line.indexOf("PRN-");
            if (pos != -1) {
                sat = YumaSatInfo();
                sat.prn = line.substring(pos + 4, pos + 6).toInt();
            }
        } else if (line.startsWith("Health:")) {
            sat.health = line.substring(line.indexOf(":") + 1).toInt();
        } else if (line.startsWith("Eccentricity:")) {
            sat.ecc = line.substring(line.indexOf(":") + 1).toDouble();
        } else if (line.startsWith("Time of Applicability(s):")) {
            sat.toa = line.substring(line.indexOf(":") + 1).toDouble();
        } else if (line.startsWith("Orbital Inclination(rad):")) {
            sat.inc = line.substring(line.indexOf(":") + 1).toDouble();
        } else if (line.startsWith("Rate of Right Ascen(r/s):")) {
            sat.rasc_rate = line.substring(line.indexOf(":") + 1).toDouble();
        } else if (line.startsWith("SQRT(A)  (m 1/2):")) {
            sat.sqrtA = line.substring(line.indexOf(":") + 1).toDouble();
        } else if (line.startsWith("Right Ascen at Week(rad):")) {
            sat.rasc = line.substring(line.indexOf(":") + 1).toDouble();
        } else if (line.startsWith("Argument of Perigee(rad):")) {
            sat.argp = line.substring(line.indexOf(":") + 1).toDouble();
        } else if (line.startsWith("Mean Anom(rad):")) {
            sat.mean_anom = line.substring(line.indexOf(":") + 1).toDouble();
        } else if (line.startsWith("Af0(s):")) {
            sat.af0 = line.substring(line.indexOf(":") + 1).toDouble();
        } else if (line.startsWith("Af1(s/s):")) {
            sat.af1 = line.substring(line.indexOf(":") + 1).toDouble();
        } else if (line.startsWith("week:")) {
            sat.week = line.substring(line.indexOf(":") + 1).toInt();
            if (sat.health == 0) yumaSats.push_back(sat);
        }
    }
    file.close();
}

// Helper: Convert GPS week/seconds to UTC time_t
static time_t gpsToUnixTime(int gpsWeek, double gpsTow) {
    // GPS epoch: Jan 6, 1980
    const time_t gpsEpoch = 315964800;
    // GPS-UTC offset (leap seconds): update as needed
    const int leapSecs = 18;
    return gpsEpoch + gpsWeek * 7 * 24 * 3600 + (int)gpsTow - leapSecs;
}

// Helper: Compute satellite ECEF position from YUMA and time (seconds since week start)
static void satEcefFromYuma(const YumaSatInfo& sat, double t, double& x, double& y, double& z) {
    // Constants
    const double mu = 3.986005e14; // Earth's universal gravitational parameter, m^3/s^2
    const double Omegae_dot = 7.2921151467e-5; // Earth rotation rate, rad/s
    // 1. Semi-major axis
    double A = sat.sqrtA * sat.sqrtA;
    // 2. Mean motion
    double n0 = sqrt(mu / (A * A * A));
    // 3. Time from ephemeris reference epoch
    double tk = t - sat.toa;
    // 4. Mean anomaly
    double M = sat.mean_anom + n0 * tk;
    // 5. Solve Kepler's equation for E (eccentric anomaly)
    double E = M;
    for (int i = 0; i < 10; ++i) E = M + sat.ecc * sin(E);
    // 6. True anomaly
    double v = atan2(sqrt(1 - sat.ecc * sat.ecc) * sin(E), cos(E) - sat.ecc);
    // 7. Argument of latitude
    double u = v + sat.argp;
    // 8. Radius
    double r = A * (1 - sat.ecc * cos(E));
    // 9. Inclination
    double i = sat.inc;
    // 10. Positions in orbital plane
    double x_orb = r * cos(u);
    double y_orb = r * sin(u);
    // 11. Corrected longitude of ascending node
    double Omega = sat.rasc + (sat.rasc_rate - Omegae_dot) * tk - Omegae_dot * sat.toa;
    // 12. ECEF coordinates
    x = x_orb * cos(Omega) - y_orb * cos(i) * sin(Omega);
    y = x_orb * sin(Omega) + y_orb * cos(i) * cos(Omega);
    z = y_orb * sin(i);
}

// Helper: Observer ECEF from lat/lon/alt
static void observerEcef(double lat, double lon, double alt, double& x, double& y, double& z) {
    const double a = 6378137.0; // WGS-84 Earth semimajor axis (m)
    const double f = 1.0 / 298.257223563;
    double sinLat = sin(lat);
    double cosLat = cos(lat);
    double sinLon = sin(lon);
    double cosLon = cos(lon);
    double e2 = f * (2 - f);
    double N = a / sqrt(1 - e2 * sinLat * sinLat);
    x = (N + alt) * cosLat * cosLon;
    y = (N + alt) * cosLat * sinLon;
    z = (N * (1 - e2) + alt) * sinLat;
}

// Helper: Compute elevation/azimuth from observer to satellite
static void topocentric(double obsX, double obsY, double obsZ, double lat, double lon, double satX, double satY, double satZ, double& elev, double& az) {
    // ENU coordinates
    double dx = satX - obsX;
    double dy = satY - obsY;
    double dz = satZ - obsZ;
    double sinLat = sin(lat);
    double cosLat = cos(lat);
    double sinLon = sin(lon);
    double cosLon = cos(lon);
    double t = -sinLon * dx + cosLon * dy;
    double e = t;
    t = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
    double n = t;
    t = cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;
    double u = t;
    double horizDist = sqrt(e * e + n * n);
    elev = atan2(u, horizDist) * 180.0 / M_PI;
    az = atan2(e, n) * 180.0 / M_PI;
    if (az < 0) az += 360.0;
}

static std::set<int> lastRealGSVPRNs;
static unsigned long lastRealGSVUpdate = 0;

// Helper: Parse PRNs from a GSV NMEA sentence
static void parseGSVPRNs(const String& nmea) {
    lastRealGSVPRNs.clear();
    int field = 0;
    int start = 0;
    for (int i = 0; i < nmea.length(); ++i) {
        if (nmea[i] == ',' || nmea[i] == '*') {
            String val = nmea.substring(start, i);
            ++field;
            // PRN fields are 4,8,12,16 (1-based)
            if (field == 4 || field == 8 || field == 12 || field == 16) {
                int prn = val.toInt();
                if (prn > 0) lastRealGSVPRNs.insert(prn);
            }
            start = i + 1;
        }
    }
    lastRealGSVUpdate = millis();
}

// Main: Generate backup GSV messages using actual visible satellites
void ICOM7100Configurator::generateBackupGSVMessages(TinyGPSPlus& gps, int satCount) {
    loadYumaAlmanacFull();
    // Get observer position (lat/lon in radians, alt in meters)
    double lat = gps.location.isValid() ? gps.location.lat() * M_PI / 180.0 : 0.0;
    double lon = gps.location.isValid() ? gps.location.lng() * M_PI / 180.0 : 0.0;
    double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    // Get time (GPS week and seconds of week)
    int gpsWeek = 0;
    double gpsTow = 0;
    if (gps.date.isValid() && gps.time.isValid()) {
        // Compute GPS week and TOW from date/time
        struct tm t = {};
        t.tm_year = gps.date.year() - 1900;
        t.tm_mon = gps.date.month() - 1;
        t.tm_mday = gps.date.day();
        t.tm_hour = gps.time.hour();
        t.tm_min = gps.time.minute();
        t.tm_sec = gps.time.second();
        time_t unixTime = mktime(&t);
        // GPS epoch: Jan 6, 1980
        const time_t gpsEpoch = 315964800;
        int totalSec = (int)(unixTime - gpsEpoch + 18); // Add leap seconds
        gpsWeek = totalSec / (7 * 24 * 3600);
        gpsTow = totalSec % (7 * 24 * 3600);
    } else if (!yumaSats.empty()) {
        gpsWeek = yumaSats[0].week;
        gpsTow = yumaSats[0].toa;
    }
    // Compute visible satellites
    struct SatView {
        int prn;
        double elev;
        double az;
    };
    std::vector<SatView> visible;
    for (const auto& sat : yumaSats) {
        double sx, sy, sz;
        satEcefFromYuma(sat, gpsTow, sx, sy, sz);
        double ox, oy, oz;
        observerEcef(lat, lon, alt, ox, oy, oz);
        double elev, az;
        topocentric(ox, oy, oz, lat, lon, sx, sy, sz, elev, az);
        if (elev > 0 && lastRealGSVPRNs.find(sat.prn) == lastRealGSVPRNs.end()) visible.push_back({sat.prn, elev, az});
    }
    // Sort by elevation descending
    std::sort(visible.begin(), visible.end(), [](const SatView& a, const SatView& b) { return a.elev > b.elev; });
    // Use up to satCount visible satellites
    int useCount = std::min(satCount, (int)visible.size());
    int numMessages = (useCount + 3) / 4;
    for (int msgNum = 1; msgNum <= numMessages; msgNum++) {
        String gsvMessage = "$GPGSV," + String(numMessages) + "," + String(msgNum) + "," + String(useCount);
        int startSat = (msgNum - 1) * 4;
        int endSat = std::min(startSat + 3, useCount - 1);
        for (int i = startSat; i <= endSat; i++) {
            int prn = visible[i].prn;
            int elevation = (int)round(visible[i].elev);
            int azimuth = (int)round(visible[i].az);
            int snr = 35 + (prn * 7) % 25; // Reasonable SNR
            gsvMessage += "," + String(prn);
            gsvMessage += "," + String(elevation);
            gsvMessage += "," + String(azimuth);
            gsvMessage += "," + String(snr);
        }
        uint8_t checksum = 0;
        for (size_t i = 1; i < gsvMessage.length(); i++) checksum ^= gsvMessage[i];
        gsvMessage += "*";
        if (checksum < 16) gsvMessage += "0";
        gsvMessage += String(checksum, HEX);
        gsvMessage += "\r\n";
        radio.print(gsvMessage);
    }
    logMessage("Generated " + String(numMessages) + " backup GSV messages with sat count: " + String(useCount));
}

void ICOM7100Configurator::generateBackupGSAMessage(TinyGPSPlus& gps, int satCount) {
    loadYumaAlmanacFull();
    double lat = gps.location.isValid() ? gps.location.lat() * M_PI / 180.0 : 0.0;
    double lon = gps.location.isValid() ? gps.location.lng() * M_PI / 180.0 : 0.0;
    double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    int gpsWeek = 0;
    double gpsTow = 0;
    if (gps.date.isValid() && gps.time.isValid()) {
        struct tm t = {};
        t.tm_year = gps.date.year() - 1900;
        t.tm_mon = gps.date.month() - 1;
        t.tm_mday = gps.date.day();
        t.tm_hour = gps.time.hour();
        t.tm_min = gps.time.minute();
        t.tm_sec = gps.time.second();
        time_t unixTime = mktime(&t);
        const time_t gpsEpoch = 315964800;
        int totalSec = (int)(unixTime - gpsEpoch + 18);
        gpsWeek = totalSec / (7 * 24 * 3600);
        gpsTow = totalSec % (7 * 24 * 3600);
    } else if (!yumaSats.empty()) {
        gpsWeek = yumaSats[0].week;
        gpsTow = yumaSats[0].toa;
    }
    struct SatView { int prn; double elev; double az; };
    std::vector<SatView> visible;
    for (const auto& sat : yumaSats) {
        double sx, sy, sz;
        satEcefFromYuma(sat, gpsTow, sx, sy, sz);
        double ox, oy, oz;
        observerEcef(lat, lon, alt, ox, oy, oz);
        double elev, az;
        topocentric(ox, oy, oz, lat, lon, sx, sy, sz, elev, az);
        if (elev > 0 && lastRealGSVPRNs.find(sat.prn) == lastRealGSVPRNs.end()) visible.push_back({sat.prn, elev, az});
    }
    std::sort(visible.begin(), visible.end(), [](const SatView& a, const SatView& b) { return a.elev > b.elev; });
    int useCount = std::min(satCount, (int)visible.size());
    String gsaMessage = "$GPGSA,A,3,";
    int maxSatsInGSA = std::min(useCount, 12);
    for (int i = 0; i < 12; i++) {
        if (i < maxSatsInGSA) {
            int prn = visible[i].prn;
            if (prn < 10) gsaMessage += "0";
            gsaMessage += String(prn) + ",";
        } else {
            gsaMessage += ",";
        }
    }
    float pdopBase = 5.0, hdopBase = 2.5, vdopBase = 4.0;
    float satFactor = 1.0 - (std::min(useCount, 12) / 24.0f);
    float pdop = pdopBase * satFactor;
    float hdop = hdopBase * satFactor;
    float vdop = vdopBase * satFactor;
    char dopStr[30];
    sprintf(dopStr, "%.1f,%.1f,%.1f", pdop, hdop, vdop);
    gsaMessage += dopStr;
    uint8_t checksum = 0;
    for (size_t i = 1; i < gsaMessage.length(); i++) checksum ^= gsaMessage[i];
    gsaMessage += "*";
    if (checksum < 16) gsaMessage += "0";
    gsaMessage += String(checksum, HEX);
    gsaMessage += "\r\n";
    radio.print(gsaMessage);
    logMessage("Generated backup GSA message with " + String(maxSatsInGSA) + " satellites");
}

// Basic Commands
void ICOM7100Configurator::setFrequency(unsigned long freq) {
    String cmd = "FA" + String(freq, DEC);
    sendCommand(cmd);
}

void ICOM7100Configurator::setMode(const String& mode) {
    String cmd = "MD" + mode;
    sendCommand(cmd);
}

void ICOM7100Configurator::setPowerLevel(int level) {
    String cmd = "PC" + String(level, DEC);
    sendCommand(cmd);
}

// GPS Related Commands
void ICOM7100Configurator::enableGPSDisplay() {
    sendCommand("GD1");
}

void ICOM7100Configurator::disableGPSDisplay() {
    sendCommand("GD0");
}

void ICOM7100Configurator::setGPSBaudRate(int rate) {
    String cmd = "GB" + String(rate, DEC);
    sendCommand(cmd);
}

// Memory Operations
void ICOM7100Configurator::recallMemory(int channel) {
    String cmd = "MR" + String(channel, DEC);
    sendCommand(cmd);
}

void ICOM7100Configurator::storeMemory(int channel) {
    String cmd = "MW" + String(channel, DEC);
    sendCommand(cmd);
}

// D-STAR Operations
void ICOM7100Configurator::setDStarCallSign(const String& callsign) {
    String cmd = "CS" + callsign;
    sendCommand(cmd);
}

void ICOM7100Configurator::setDStarMessage(const String& message) {
    String cmd = "MS" + message;
    sendCommand(cmd);
}

// Scan Operations
void ICOM7100Configurator::startScan() {
    sendCommand("SC");
}

void ICOM7100Configurator::stopScan() {
    sendCommand("SC0");
}

// Voice Memory Operations
void ICOM7100Configurator::recordVoiceMemory(int channel) {
    String cmd = "VR" + String(channel, DEC);
    sendCommand(cmd);
}

void ICOM7100Configurator::playVoiceMemory(int channel) {
    String cmd = "VP" + String(channel, DEC);
    sendCommand(cmd);
}

// Control Commands
void ICOM7100Configurator::setSquelch(int level) {
    String cmd = "SQ" + String(level, DEC);
    sendCommand(cmd);
}

void ICOM7100Configurator::setVolume(int level) {
    String cmd = "AG" + String(level, DEC);
    sendCommand(cmd);
}

// GPS-A Operations
void ICOM7100Configurator::enableGPSA() {
    sendCommand("GA1");
}

void ICOM7100Configurator::disableGPSA() {
    sendCommand("GA0");
}

// Status Query
bool ICOM7100Configurator::queryStatus() {
    // Send status query command
    uint8_t statusCmd[] = {0xFE, 0xFE, 0x88, 0xE0, 0x03, 0xFD};
    radio.write(statusCmd, sizeof(statusCmd));
    
    
    // Wait for response
    delay(100);
    
    // Read response
    uint8_t response[8];
    int bytesRead = 0;
    unsigned long startTime = millis();
    while (millis() - startTime < 500 && bytesRead < sizeof(response)) {
        if (radio.available()) {
            response[bytesRead++] = radio.read();
        }
    }
    
    // Check if response is valid (FE FE E0 88 03 XX FD)
    if (bytesRead >= 7 && 
        response[0] == 0xFE && 
        response[1] == 0xFE && 
        response[2] == 0xE0 && 
        response[3] == 0x88 && 
        response[4] == 0x03 && 
        response[6] == 0xFD) {
        logMessage("Radio status query successful");
        return true;
    }
    
    logMessage("Radio status query failed");
    return false;
}

// Initialize radio with default settings
void ICOM7100Configurator::initialize() {
    // Set default GPS baud rate to match our GPS module
    setGPSBaudRate(9600);
    
    // Enable GPS display
    enableGPSDisplay();
    
    // Set default volume
    setVolume(50);
    
    // Set default squelch
    setSquelch(20);

    // Enable GPS-A functionality
    enableGPSA();

}

void ICOM7100Configurator::reportGPSStats() {
    unsigned long now = millis();
    if (now - lastStatsReportTime >= STATS_REPORT_INTERVAL) {
        String statsMessage = "GPS Stats - GGA: " + String(ggaMessagesSent) + 
                            ", RMC: " + String(rmcMessagesSent) + 
                            ", Errors: " + String(messageErrors) +
                            ", Null: " + String(nullMessages) +
                            ", Converted: " + String(convertedMessages);
        logMessage(statsMessage);
        
        // Reset counters
        ggaMessagesSent = 0;
        rmcMessagesSent = 0;
        messageErrors = 0;
        nullMessages = 0;
        convertedMessages = 0;
        lastStatsReportTime = now;
    }
}

// UsbRadio Implementation
UsbRadio::UsbRadio(HardwareSerial& radioUart) : radio(radioUart) {}

void UsbRadio::begin() {
    // No special setup needed for now
}

void UsbRadio::loop() {
    // Bridge USB CDC <-> radio UART
    // Forward USB->Radio
    while (Serial.available()) {
        int c = Serial.read();
        radio.write((uint8_t)c);
    }
    // Forward Radio->USB
    while (radio.available()) {
        int c = radio.read();
        Serial.write((uint8_t)c);
    }
}

void UsbRadio::sendCATCommand(const uint8_t* data, size_t len) {
    radio.write(data, len);
}

void UsbRadio::sendDVData(const uint8_t* data, size_t len) {
    radio.write(data, len);
}

bool UsbRadio::isConnected() {
    static unsigned long lastCheck = 0;
    static bool lastResult = false;
    unsigned long now = millis();
    if (now - lastCheck < 2000) {
        return lastResult;
    }
    lastCheck = now;
    // Send a simple ICOM status query (e.g., "FE FE 88 E0 03 FD")
    uint8_t statusCmd[] = {0xFE, 0xFE, 0x88, 0xE0, 0x03, 0xFD};
    Serial.write(statusCmd, sizeof(statusCmd));
    delay(100); // Wait for response
    uint8_t response[8];
    int bytesRead = 0;
    unsigned long startTime = millis();
    while (millis() - startTime < 200 && bytesRead < sizeof(response)) {
        if (Serial.available()) {
            response[bytesRead++] = Serial.read();
        }
    }
    // Check for valid ICOM response (FE FE E0 88 03 XX FD)
    if (bytesRead >= 7 &&
        response[0] == 0xFE &&
        response[1] == 0xFE &&
        response[2] == 0xE0 &&
        response[3] == 0x88 &&
        response[4] == 0x03 &&
        response[6] == 0xFD) {
        lastResult = true;
    } else {
        lastResult = false;
    }
    return lastResult;
} 