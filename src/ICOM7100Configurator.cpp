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

// Define the YumaSatInfo struct first
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

// Forward declarations for helper functions
static void loadYumaAlmanacFull();
static time_t gpsToUnixTime(int gpsWeek, double gpsTow);
static void satEcefFromYuma(const YumaSatInfo& sat, double t, double& x, double& y, double& z);
static void observerEcef(double lat, double lon, double alt, double& x, double& y, double& z);
static void topocentric(double obsX, double obsY, double obsZ, double lat, double lon, double satX, double satY, double satZ, double& elev, double& az);

// Constructor implementation
ICOM7100Configurator::ICOM7100Configurator(HardwareSerial& serial) 
    : radio(serial), lastCommandTime(0), satelliteManager() {
    // Initialize backup data variables
    _usingBackupData = false;
    ggaMessagesSent = 0;
    rmcMessagesSent = 0;
    messageErrors = 0;
    nullMessages = 0;
    convertedMessages = 0;
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

// Helper to convert any NMEA prefix to $GP for ICOM 7100 compatibility, and recalculate checksum
auto convertToGPPrefix = [](const String& nmea) -> String {
    return nmea; // Simply return the original message without any conversion
};

// Helper function to create a null NMEA sentence (for when data is missing)
String ICOM7100Configurator::nullNMEA(const String& prefix) {
    // $GPGGA,,,,,,0,,,,,,,,*
    if (prefix == "$GPGGA") {
        return "$GPGGA,000000.00,,,,,0,00,99.9,0,M,0,M,,*66";
    }
    // $GPRMC,,V,,,,,,,,,,N*
    else if (prefix == "$GPRMC") {
        return "$GPRMC,000000.00,V,,,,,,,,,,,N*53";
    }
    // $GPGSA,A,1,,,,,,,,,,,,,99.9,99.9,99.9*
    else if (prefix == "$GPGSA") {
        return "$GPGSA,A,1,,,,,,,,,,,,,99.9,99.9,99.9*30";
    }
    // Unknown type, return empty
    return "";
}

// Add a new method to ICOM7100Configurator to generate fixed-satellite GSA messages
String ICOM7100Configurator::generateFixedGSAMessage() {
    // Format a GSA message with satellites we know are used
    // We'll use a specific set of satellites (if we have them)
    std::vector<int> prioritySats = satelliteManager.getUsedSatellites();
    
    // If we have less than 2 satellites, add dummy sats to ensure display
    if (prioritySats.size() < 2) {
        // Add some standard GPS PRNs 
        if (std::find(prioritySats.begin(), prioritySats.end(), 1) == prioritySats.end())
            prioritySats.push_back(1);
        if (std::find(prioritySats.begin(), prioritySats.end(), 5) == prioritySats.end())
            prioritySats.push_back(5);
        if (std::find(prioritySats.begin(), prioritySats.end(), 9) == prioritySats.end())
            prioritySats.push_back(9);
    }
    
    // Sort satellites for consistent display
    std::sort(prioritySats.begin(), prioritySats.end());
    
    // Limit to 12 satellites (maximum for GSA)
    if (prioritySats.size() > 12) {
        prioritySats.resize(12);
    }
    
    // Format the GSA message
    String gsa = "$GPGSA,A,3"; // Auto selection, 3D fix
    
    // Add satellite PRNs (up to 12)
    for (int i = 0; i < 12; i++) {
        if (i < prioritySats.size()) {
            gsa += "," + String(prioritySats[i]);
        } else {
            gsa += ","; // Empty field for unused slots
        }
    }
    
    // Add DOP values - calculate based on number of satellites
    float pdop = 2.5, hdop = 1.5, vdop = 2.0;
    if (prioritySats.size() >= 10) {
        pdop = 1.5;
        hdop = 0.9;
        vdop = 1.2;
    } else if (prioritySats.size() >= 8) {
        pdop = 1.8;
        hdop = 1.1;
        vdop = 1.5;
    } else if (prioritySats.size() >= 6) {
        pdop = 2.2;
        hdop = 1.3;
        vdop = 1.8;
    }
    
    // Add DOP values
    char dopStr[30];
    sprintf(dopStr, "%.1f,%.1f,%.1f", pdop, hdop, vdop);
    gsa += "," + String(dopStr);
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 1; i < gsa.length(); i++) {
        checksum ^= gsa[i];
    }
    
    // Add checksum
    gsa += "*";
    if (checksum < 16) gsa += "0";
    gsa += String(checksum, HEX);
    gsa.toUpperCase();
    
    // Add proper line ending for NMEA messages
    gsa += "\r\n";
    
    return gsa;
}

// Main function to forward NMEA data to the radio
void ICOM7100Configurator::forwardNMEAToRadio(TinyGPSPlus& gps, int altitudeCorrection, unsigned long charsPerSecond) {
    // Static variables for tracking sent messages
    static String lastSentGGA = "";
    static String lastSentRMC = "";
    static String lastSentGSA = "";
    static bool everReceivedRealSatData = false;
    static unsigned long lastRealMessageTime = 0;
    static unsigned long lastSendTime = 0;
    static unsigned long lastStatsTime = 0;
    static unsigned long startupTime = millis(); // Track time since startup
    static bool initialSatDataSent = false;  // Flag to track if we've sent initial sat data

    // Reference to external globals from main.cpp
    extern std::vector<String> gsvBuffer;
    extern String latestGGA;
    extern String latestRMC;
    extern String latestGSA;
    extern bool hasValidGSVs;
    
    unsigned long now = millis();
    int totalSats = gps.satellites.isValid() ? gps.satellites.value() : 0;

    // Ensure the GGA message has the correct satellite count field
    if (latestGGA.length() > 0) {
        // Parse the GGA message and replace the satellite count field (7th field)
        int firstComma = latestGGA.indexOf(',');
        int commaCount = 0;
        int startIdx = 0;
        int endIdx = 0;
        String patchedGGA = "";
        
        // Find the position of the 7th field (satellite count)
        for (int i = 0; i < latestGGA.length(); i++) {
            if (latestGGA[i] == ',') {
                commaCount++;
                if (commaCount == 6) {
                    startIdx = i + 1;
                } else if (commaCount == 7) {
                    endIdx = i;
                    break;
                }
            }
        }
        
        if (startIdx > 0 && endIdx > startIdx) {
            // Replace the satellite count field with the actual value
            String beforeSatCount = latestGGA.substring(0, startIdx);
            String afterSatCount = latestGGA.substring(endIdx);
            
            // Format the sat count with leading zero if needed
            String satCountStr = String(totalSats);
            if (totalSats < 10) {
                satCountStr = "0" + satCountStr;
            }
            
            patchedGGA = beforeSatCount + satCountStr + afterSatCount;
            
            // Recalculate the checksum
            int starIdx = patchedGGA.indexOf('*');
            if (starIdx > 0) {
                String body = patchedGGA.substring(1, starIdx);
        uint8_t checksum = 0;
                for (size_t i = 0; i < body.length(); i++) {
                    checksum ^= body[i];
                }
                
                // Format the checksum
                String hexChecksum = "";
                if (checksum < 16) hexChecksum += "0";
                hexChecksum += String(checksum, HEX);
                
                patchedGGA = patchedGGA.substring(0, starIdx + 1) + hexChecksum;
                if (patchedGGA.endsWith("\r\n") == false) {
                    patchedGGA += "\r\n";
                }
                
                // Update the latestGGA with the patched version
                latestGGA = patchedGGA;
            }
        }
    }

    // Process new GSV messages into our satellite manager
    if (hasValidGSVs && !gsvBuffer.empty()) {
        bool processedAny = false;
        for (const auto& gsv : gsvBuffer) {
            if (satelliteManager.processGSVMessage(gsv)) {
                processedAny = true;
                everReceivedRealSatData = true;
                lastRealMessageTime = now;
            }
        }
        
        // If we processed any valid GSV messages, update our satellite count
        if (processedAny) {
            // Update totalSats based on the satellite manager if it's higher
            int visibleSats = satelliteManager.getVisibleSatellites();
            if (visibleSats > totalSats) {
                totalSats = visibleSats;
            }
            
            // During startup, force more frequent updates to get satellites showing quickly
            if (now - startupTime < 60000 && !initialSatDataSent) {
                // Force a transmission update at startup
                satelliteManager.forceTransmit();
                initialSatDataSent = true;
            }
        }
    }
    
    // Process GSA message for satellite usage information
    if (latestGSA.length() > 0) {
        satelliteManager.processGSAMessage(latestGSA);
    }

    // Use backup data if necessary
    if (!gps.location.isValid() || (now - lastRealMessageTime > BACKUP_DATA_TIMEOUT)) {
        // Check if we've ever received real satellite data
        if (everReceivedRealSatData && gps.location.isValid()) {
            // Generate backup GSV and GSA messages based on last known values
            generateBackupGSVMessages(gps, totalSats);
            generateBackupGSAMessage(gps, totalSats);
            _usingBackupData = true;
        } else {
            _usingBackupData = false;
        }
    } else {
        _usingBackupData = false;
    }
    
    // Only send messages every RADIO_SEND_INTERVAL ms
    if (now - lastSendTime > RADIO_SEND_INTERVAL) {
        // Check if there are any satellites in our database
        // If not, delay sending other NMEA messages to prevent the radio from showing 0 satellites
        int visibleSats = satelliteManager.getVisibleSatellites();
        bool haveSatelliteData = (visibleSats > 0) || initialSatDataSent;
        
        // During startup, force a satellite check every send interval until we have data
        if (!haveSatelliteData && now - startupTime < 15000) {
            // Startup mode - generate at least some dummy data so we see satellites
            for (int i = 1; i <= 6; i++) {
                // Create dummy satellite entries with reasonable values
                SatelliteInfo dummySat(i, 30 + i * 5, 60 + i * 30, 20 + i);
                satelliteManager.processGSVMessage("$GPGSV,1,1,6," + 
                    String(i) + "," + String(dummySat.elevation) + "," + 
                    String(dummySat.azimuth) + "," + String(dummySat.snr));
            }
            haveSatelliteData = true;
            satelliteManager.forceTransmit();
        }
        
        // Always send the GSV (satellite position) messages first to ensure satellites appear on map
        // Before sending other messages like GGA and GSA
        
        // GSV - For satellite position and ID data - now using SatelliteDataManager
        // During startup, force satellite data to be sent
        bool forceUpdate = !initialSatDataSent && (now - startupTime < 30000);
        
        if (satelliteManager.shouldTransmit() || _usingBackupData || forceUpdate) {
            std::vector<String> gsvMessages = satelliteManager.generateGSVMessages();
            
            if (!gsvMessages.empty()) {
                // Send the generated GSV messages
                for (const auto& gsv : gsvMessages) {
                    String out = gsv;
                    if (!out.endsWith("\r\n")) out += "\r\n";
                    logMessage("SENDING TO RADIO: " + out);
                    radio.print(out);
                    convertedMessages++;
                    delay(10); // Small delay between GSV messages
                }
                
                satelliteManager.markTransmitted();
                initialSatDataSent = true; // Mark that we've sent initial data
                
                logMessage("Updated satellite display with " + String(gsvMessages.size()) + 
                          " GSV messages and " + String(satelliteManager.getVisibleSatellites()) + 
                          " satellites");
                
                // Save these messages for backup use
                _backupGSVs = gsvMessages;
            }
            // If no valid satellites from manager but in backup mode, generate fallback
            else if (_usingBackupData) {
                // Generate backup GSV messages (when real data is briefly unavailable)
                logMessage("Generating backup GSV data using last known satellites");
                
                if (!_backupGSVs.empty()) {
                    // Send the last known good GSV messages
                    for (const auto& gsv : _backupGSVs) {
                        String out = gsv;
                        if (!out.endsWith("\r\n")) out += "\r\n";
                        logMessage("SENDING BACKUP GSV: " + out);
                        radio.print(out);
                        delay(10);
                    }
                } else {
                    // If we have no last known GSVs, generate some based on actual GPS data
                    generateBackupGSVMessages(gps, totalSats > 0 ? totalSats : 6);
                }
            }
        }
        else if (hasValidGSVs) {
            logMessage("Skipping GSV update, keeping stable satellite display (" + 
                      String(satelliteManager.getVisibleSatellites()) + " satellites)");
        }
        else {
            logMessage("No valid satellite data in GSV messages, skipping");
        }
        
        // Now send GSA message - always generate our own GSA with the satellites actually used
        String fixedGSA = generateFixedGSAMessage();
        if (fixedGSA.length() > 0) {
            logMessage("SENDING TO RADIO (FIXED-SAT GSA): " + fixedGSA);
            radio.print(fixedGSA);
            lastSentGSA = fixedGSA;
            convertedMessages++;
        }
        else {
            // Fall back to normal GSA handling if our fixed GSA generator fails
            String generatedGSA = satelliteManager.generateGSAMessage();
            if (generatedGSA.length() > 0) {
                logMessage("SENDING TO RADIO: " + generatedGSA);
                radio.print(generatedGSA);
                lastSentGSA = generatedGSA;
                convertedMessages++;
            } else if (latestGSA.length() > 0 && latestGSA != lastSentGSA) {
                // Fallback to original GSA message if the manager couldn't generate one
                String out = latestGSA;
                logMessage("SENDING TO RADIO: " + out);
                radio.print(out);
                lastSentGSA = latestGSA;
            } else if (latestGSA.length() == 0 && generatedGSA.length() == 0) {
                String nullMsg = nullNMEA("$GPGSA");
                logMessage("SENDING TO RADIO: " + nullMsg);
                radio.print(nullMsg);
                nullMessages++;
                lastSentGSA = "";
            }
        }

        // GGA - For lat/lon/alt/time
        if (latestGGA.length() > 0 && latestGGA != lastSentGGA) {
            String out = latestGGA;
            
            // Ensure the satellite count in the GGA message matches what the GPS reports
            // Format: $GPGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,geoid,M,age,refid*CS
            int fixField = out.indexOf(',', 6) + 1; // Start after $GPGGA,
            for (int i = 0; i < 5; i++) {
                fixField = out.indexOf(',', fixField) + 1;
            }
            int satField = out.indexOf(',', fixField) + 1;
            int nextComma = out.indexOf(',', satField);
            
            if (fixField > 0 && satField > 0 && nextComma > 0) {
                // Extract the existing fix type
                String fixType = out.substring(fixField, satField - 1);
                int fix = fixType.toInt();
                
                // Get the number of satellites reported by the GPS
                int reportedSats = satelliteManager.getVisibleSatellites();
                if (reportedSats <= 0) reportedSats = totalSats; // Use total from GSV messages if manager doesn't have data
                
                // Always ensure at least a minimum number of satellites is shown
                if (reportedSats < 4) reportedSats = 4; // Minimum satellites for display
                
                // Replace the satellite count field
                String satCount = String(reportedSats);
                String beforeSat = out.substring(0, satField);
                String afterSat = out.substring(nextComma);
                out = beforeSat + satCount + afterSat;
                
                // If fix is 0 or empty, set it to 1 (at least GPS fix)
                if (fix <= 0) {
                    int fixEnd = satField - 1;
                    String beforeFix = out.substring(0, fixField);
                    String afterFix = out.substring(fixEnd);
                    out = beforeFix + "1" + afterFix;
                }
                
                // Recalculate checksum
                int starIdx = out.indexOf('*');
                if (starIdx > 0) {
                    String body = out.substring(1, starIdx);
                    uint8_t checksum = 0;
                    for (size_t i = 0; i < body.length(); i++) {
                        checksum ^= body[i];
                    }
                    String hexChecksum = "";
                    if (checksum < 16) hexChecksum += "0";
                    hexChecksum += String(checksum, HEX);
                    out = out.substring(0, starIdx + 1) + hexChecksum;
                }
            }
            
            if (!out.endsWith("\r\n")) out += "\r\n";
            logMessage("SENDING TO RADIO: " + out);
            radio.print(out);
            ggaMessagesSent++;
            lastSentGGA = latestGGA;
            _lastGGA = out; // Save for backup use
        } else if (latestGGA.length() == 0) {
            String nullMsg = nullNMEA("$GPGGA");
            if (!nullMsg.endsWith("\r\n")) nullMsg += "\r\n";
            logMessage("SENDING TO RADIO: " + nullMsg);
            radio.print(nullMsg);
            nullMessages++;
            lastSentGGA = "";
        }
        
        // RMC
        if (latestRMC.length() > 0 && latestRMC != lastSentRMC) {
            String out = latestRMC;
            if (!out.endsWith("\r\n")) out += "\r\n";
            logMessage("SENDING TO RADIO: " + out);
            radio.print(out);
            rmcMessagesSent++;
            lastSentRMC = latestRMC;
            _lastRMC = out; // Save for backup use
        } else if (latestRMC.length() == 0) {
            String nullMsg = nullNMEA("$GPRMC");
            if (!nullMsg.endsWith("\r\n")) nullMsg += "\r\n";
            logMessage("SENDING TO RADIO: " + nullMsg);
            radio.print(nullMsg);
            nullMessages++;
            lastSentRMC = "";
        }
        
        lastSendTime = now;
        
        // Report stats every 10 seconds
        if (now - lastStatsTime > STATS_REPORT_INTERVAL) {
            reportGPSStats(satelliteManager.getVisibleSatellites(), charsPerSecond);
            lastStatsTime = now;
        }
    }
}

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
    // Create an empty set since we're not tracking PRNs now
    std::set<int> emptyPRNs;
    
    for (const auto& sat : yumaSats) {
        double sx, sy, sz;
        satEcefFromYuma(sat, gpsTow, sx, sy, sz);
        double ox, oy, oz;
        observerEcef(lat, lon, alt, ox, oy, oz);
        double elev, az;
        topocentric(ox, oy, oz, lat, lon, sx, sy, sz, elev, az);
        if (elev > 0) visible.push_back({sat.prn, elev, az});
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
        logMessage("SENDING TO RADIO: " + gsvMessage);
        radio.print(gsvMessage);
        convertedMessages++;
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
        if (elev > 0) visible.push_back({sat.prn, elev, az});
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
    logMessage("SENDING TO RADIO: " + gsaMessage);
    radio.print(gsaMessage);
    convertedMessages++;
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

void ICOM7100Configurator::reportGPSStats(int satCount, unsigned long charsPerSecond) {
    unsigned long now = millis();
    if (now - lastStatsReportTime >= STATS_REPORT_INTERVAL) {
        String statsMessage = "GPS Stats - GGA: " + String(ggaMessagesSent) + 
                            ", RMC: " + String(rmcMessagesSent) + 
                            ", Errors: " + String(messageErrors) +
                            ", Null: " + String(nullMessages) +
                            ", Converted: " + String(convertedMessages) +
                            ", Sats: " + String(satCount) +
                            ", Chars/s: " + String(charsPerSecond);
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