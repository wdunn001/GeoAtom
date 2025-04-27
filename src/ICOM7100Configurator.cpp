#include "ICOM7100Configurator.h"

// Constructor implementation
ICOM7100Configurator::ICOM7100Configurator(HardwareSerial& serial) 
    : radioSerial(serial), lastCommandTime(0) {
}

// Helper function to send command with checksum
void ICOM7100Configurator::sendCommand(const String& cmd) {
    if (millis() - lastCommandTime < COMMAND_DELAY) {
        delay(COMMAND_DELAY);
    }
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 0; i < cmd.length(); i++) {
        checksum ^= cmd[i];
    }
    
    // Log the command being sent
    logMessage ("\n[Radio] Sending command: ");
    logMessage(cmd);
    if (checksum < 16) logMessage("0");
    logMessage(String(checksum, HEX));
    logMessage("\r\n");
    
    // Send command with checksum
    radioSerial.print(cmd);
    if (checksum < 16) radioSerial.print("0");
    radioSerial.print(checksum, HEX);
    radioSerial.print("\r\n");
    
    lastCommandTime = millis();
}

// Helper function to calculate NMEA checksum
uint8_t calculateNMEAChecksum(const String& message) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < message.length(); i++) {
        checksum ^= message[i];
    }
    return checksum;
}

// Convert raw GPS data to GGA message
String createGGAMessage(float lat, float lng, float alt, int sats, float hdop, int hour, int minute, int second) {
    String ggaMessage = "$GPGGA,";
    
    // Time
    ggaMessage += String(hour) + String(minute) + String(second) + ".00,";
    
    // Latitude
    ggaMessage += String(abs(lat), 4) + (lat < 0 ? "S," : "N,");
    
    // Longitude
    ggaMessage += String(abs(lng), 4) + (lng < 0 ? "W," : "E,");
    
    // Fix quality (1 = GPS fix)
    ggaMessage += "1,";
    
    // Number of satellites
    ggaMessage += String(sats) + ",";
    
    // HDOP
    ggaMessage += String(hdop, 1) + ",";
    
    // Altitude
    ggaMessage += String(alt, 1) + ",M,";
    
    // Geoid separation and age of diff
    ggaMessage += "0.0,M,,";

    // Calculate and append checksum
    uint8_t checksum = calculateNMEAChecksum(ggaMessage);
    ggaMessage += "*";
    if (checksum < 16) ggaMessage += "0";
    ggaMessage += String(checksum, HEX);
    ggaMessage += "\r\n";

    return ggaMessage;
}

// Convert raw GPS data to RMC message
String createRMCMessage(float lat, float lng, float speed, float course, int hour, int minute, int second, int day, int month, int year) {
    String rmcMessage = "$GPRMC,";
    
    // Time
    rmcMessage += String(hour) + String(minute) + String(second) + ".00,";
    
    // Status (A=active)
    rmcMessage += "A,";
    
    // Latitude
    rmcMessage += String(abs(lat), 4) + (lat < 0 ? "S," : "N,");
    
    // Longitude
    rmcMessage += String(abs(lng), 4) + (lng < 0 ? "W," : "E,");
    
    // Speed in knots
    rmcMessage += String(speed, 1) + ",";
    
    // Course in degrees
    rmcMessage += String(course, 1) + ",";
    
    // Date
    rmcMessage += String(day) + String(month) + String(year % 100) + ",";
    
    // Magnetic variation
    rmcMessage += "0.0,E,";

    // Calculate and append checksum
    uint8_t checksum = calculateNMEAChecksum(rmcMessage);
    rmcMessage += "*";
    if (checksum < 16) rmcMessage += "0";
    rmcMessage += String(checksum, HEX);
    rmcMessage += "\r\n";

    return rmcMessage;
}

// Convert VTG message to RMC components
void parseVTGToRMC(const String& vtgMessage, float& speed, float& course) {
    // VTG format: $GPVTG,course,T,course,M,speed,N,speed,K*checksum
    int firstComma = vtgMessage.indexOf(',');
    int secondComma = vtgMessage.indexOf(',', firstComma + 1);
    int thirdComma = vtgMessage.indexOf(',', secondComma + 1);
    int fourthComma = vtgMessage.indexOf(',', thirdComma + 1);
    
    if (firstComma != -1 && secondComma != -1 && thirdComma != -1 && fourthComma != -1) {
        course = vtgMessage.substring(firstComma + 1, secondComma).toFloat();
        speed = vtgMessage.substring(thirdComma + 1, fourthComma).toFloat();
    }
}

// Convert GSA message to GGA components
void parseGSAToGGA(const String& gsaMessage, int& sats, float& hdop) {
    // GSA format: $GPGSA,mode,fix,sat1,sat2,...,pdop,hdop,vdop*checksum
    int lastComma = gsaMessage.lastIndexOf(',');
    int secondLastComma = gsaMessage.lastIndexOf(',', lastComma - 1);
    
    if (lastComma != -1 && secondLastComma != -1) {
        hdop = gsaMessage.substring(secondLastComma + 1, lastComma).toFloat();
        
        // Count number of satellites
        sats = 0;
        int start = gsaMessage.indexOf(',', gsaMessage.indexOf(',') + 1) + 1;
        int end = gsaMessage.indexOf(',', start);
        while (end != -1 && sats < 12) {
            if (gsaMessage.substring(start, end).toInt() > 0) {
                sats++;
            }
            start = end + 1;
            end = gsaMessage.indexOf(',', start);
        }
    }
}

// Convert GSV message to GGA components
void parseGSVToGGA(const String& gsvMessage, int& sats) {
    // GSV format: $GPGSV,totalMsgs,msgNum,totalSats,sat1,elev1,azim1,snr1,...*checksum
    int thirdComma = gsvMessage.indexOf(',', gsvMessage.indexOf(',', gsvMessage.indexOf(',') + 1) + 1);
    if (thirdComma != -1) {
        int fourthComma = gsvMessage.indexOf(',', thirdComma + 1);
        if (fourthComma != -1) {
            sats = gsvMessage.substring(thirdComma + 1, fourthComma).toInt();
        }
    }
}

// Convert GLL message to GGA and RMC components
void parseGLLToComponents(const String& gllMessage, float& lat, float& lng, int& hour, int& minute, int& second) {
    // GLL format: $GPGLL,lat,N/S,lng,E/W,time,A*checksum
    int firstComma = gllMessage.indexOf(',');
    int secondComma = gllMessage.indexOf(',', firstComma + 1);
    int thirdComma = gllMessage.indexOf(',', secondComma + 1);
    int fourthComma = gllMessage.indexOf(',', thirdComma + 1);
    int fifthComma = gllMessage.indexOf(',', fourthComma + 1);
    
    if (firstComma != -1 && secondComma != -1 && thirdComma != -1 && fourthComma != -1 && fifthComma != -1) {
        // Parse latitude
        String latStr = gllMessage.substring(firstComma + 1, secondComma);
        lat = latStr.toFloat();
        if (gllMessage.charAt(secondComma + 1) == 'S') lat = -lat;
        
        // Parse longitude
        String lngStr = gllMessage.substring(thirdComma + 1, fourthComma);
        lng = lngStr.toFloat();
        if (gllMessage.charAt(fourthComma + 1) == 'W') lng = -lng;
        
        // Parse time
        String timeStr = gllMessage.substring(fifthComma + 1, gllMessage.indexOf('.', fifthComma));
        hour = timeStr.substring(0, 2).toInt();
        minute = timeStr.substring(2, 4).toInt();
        second = timeStr.substring(4, 6).toInt();
    }
}

// NMEA Forwarding
void ICOM7100Configurator::forwardNMEAToRadio(TinyGPSPlus& gps, int altitudeCorrection) {
    static unsigned long lastForwardTime = 0;
    const unsigned long FORWARD_INTERVAL = 500; // Forward every 100ms

    if (millis() - lastForwardTime >= FORWARD_INTERVAL) {
        // Debug log GPS status
        logMessage("\n=== GPS Status ===");
        logMessage("Location Valid: " + String(gps.location.isValid() ? "YES" : "NO"));
        logMessage("Time Valid: " + String(gps.time.isValid() ? "YES" : "NO"));
        logMessage("Date Valid: " + String(gps.date.isValid() ? "YES" : "NO"));
        logMessage("Satellites: " + String(gps.satellites.value()));
        logMessage("HDOP: " + String(gps.hdop.hdop(), 1));

        // If we have valid data, create and send messages
        if (gps.location.isValid()) {
            // Create GGA message
            String ggaMessage = "$GPGGA,";
            ggaMessage += String(gps.time.hour()) + String(gps.time.minute()) + String(gps.time.second()) + ".00,";
            ggaMessage += String(abs(gps.location.lat()), 4) + (gps.location.lat() < 0 ? "S," : "N,");
            ggaMessage += String(abs(gps.location.lng()), 4) + (gps.location.lng() < 0 ? "W," : "E,");
            ggaMessage += "1,"; // Fix quality
            ggaMessage += String(gps.satellites.value()) + ",";
            ggaMessage += String(gps.hdop.hdop(), 1) + ",";
            ggaMessage += String(gps.altitude.meters() + altitudeCorrection, 1) + ",M,";
            ggaMessage += "0.0,M,,";

            // Calculate checksum
            uint8_t checksum = 0;
            for (size_t i = 1; i < ggaMessage.length(); i++) {
                checksum ^= ggaMessage[i];
            }
            ggaMessage += "*";
            if (checksum < 16) ggaMessage += "0";
            ggaMessage += String(checksum, HEX);
            ggaMessage += "\r\n";

            // Debug log GGA message
            logMessage("\n=== Sending GGA Message ===");
            logMessage(ggaMessage);

            // Send GGA to Radio
            radioSerial.print(ggaMessage);
            logMessage("GGA sent to radio");

            // Create RMC message
            String rmcMessage = "$GPRMC,";
            rmcMessage += String(gps.time.hour()) + String(gps.time.minute()) + String(gps.time.second()) + ".00,";
            rmcMessage += "A,"; // Status
            rmcMessage += String(abs(gps.location.lat()), 4) + (gps.location.lat() < 0 ? "S," : "N,");
            rmcMessage += String(abs(gps.location.lng()), 4) + (gps.location.lng() < 0 ? "W," : "E,");
            rmcMessage += String(gps.speed.knots(), 1) + ",";
            rmcMessage += String(gps.course.deg(), 1) + ",";
            rmcMessage += String(gps.date.day()) + String(gps.date.month()) + String(gps.date.year() % 100) + ",";
            rmcMessage += "0.0,E,";

            // Calculate checksum
            checksum = 0;
            for (size_t i = 1; i < rmcMessage.length(); i++) {
                checksum ^= rmcMessage[i];
            }
            rmcMessage += "*";
            if (checksum < 16) rmcMessage += "0";
            rmcMessage += String(checksum, HEX);
            rmcMessage += "\r\n";

            // Debug log RMC message
            logMessage("\n=== Sending RMC Message ===");
            logMessage(rmcMessage);

            // Send RMC to Radio
            radioSerial.print(rmcMessage);
            logMessage("RMC sent to radio");

            // Check for radio response
            if (radioSerial.available()) {
                logMessage("\n=== Radio Response ===");
                String response = "";
                while (radioSerial.available()) {
                    response += (char)radioSerial.read();
                }
                logMessage(response);
            }
        } else {
            logMessage("No valid GPS data to send");
        }
        
        logMessage("==================");
        lastForwardTime = millis();
    }
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

// Initialize radio with default settings
void ICOM7100Configurator::initialize() {
    // Set default GPS baud rate to match our GPS module
    setGPSBaudRate(4800);
    
    // Enable GPS display
    enableGPSDisplay();
    
    // Set default volume
    setVolume(50);
    
    // Set default squelch
    setSquelch(20);

    // Enable GPS-A functionality
    enableGPSA();
} 