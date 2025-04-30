#include "ICOM7100Configurator.h"

// Constructor implementation
ICOM7100Configurator::ICOM7100Configurator(HardwareSerial& serial) 
    : radio(serial), lastCommandTime(0), usbModeEnabled(false) {
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
    
    // If USB mode is enabled, also send to Serial
    if (usbModeEnabled) {
        Serial.print(cmd);
        if (checksum < 16) Serial.print("0");
        Serial.print(checksum, HEX);
        Serial.print("\r\n");
    }
    
    lastCommandTime = millis();
}

// NMEA Forwarding
void ICOM7100Configurator::forwardNMEAToRadio(TinyGPSPlus& gps, int altitudeCorrection) {
    static unsigned long lastSendTime = 0;
    const unsigned long FORWARD_INTERVAL = 500; // 2Hz

    unsigned long now = millis();
    if (now - lastSendTime >= FORWARD_INTERVAL) {
        bool hasValidData = gps.location.isValid() && gps.time.isValid();

        // Always send GGA message with consistent format
        String ggaMessage = "$GPGGA,";
        
        // Time field - always in same format
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
            ggaMessage += "000000.00,";
        }

        // Position and quality fields - always in same format
        if (gps.location.isValid()) {
            // Format latitude as DDMM.MMMM
            float lat = abs(gps.location.lat());
            int latDeg = (int)lat;
            float latMin = (lat - latDeg) * 60.0;
            String latDegStr = String(latDeg);
            if (latDeg < 10) latDegStr = "0" + latDegStr;
            String latMinStr = String(latMin, 4);
            while (latMinStr.length() < 7) latMinStr = "0" + latMinStr; // Ensure 4 decimal places
            ggaMessage += latDegStr + latMinStr + (gps.location.lat() < 0 ? ",S," : ",N,");

            // Format longitude as DDDMM.MMMM
            float lng = abs(gps.location.lng());
            int lngDeg = (int)lng;
            float lngMin = (lng - lngDeg) * 60.0;
            String lngDegStr = String(lngDeg);
            if (lngDeg < 100) lngDegStr = "0" + lngDegStr;
            if (lngDeg < 10) lngDegStr = "0" + lngDegStr;
            String lngMinStr = String(lngMin, 4);
            while (lngMinStr.length() < 7) lngMinStr = "0" + lngMinStr; // Ensure 4 decimal places
            ggaMessage += lngDegStr + lngMinStr + (gps.location.lng() < 0 ? ",W," : ",E,");

            // Quality indicator (1 = GPS fix)
            ggaMessage += "1,";
            
            // Number of satellites
            String sats = String(gps.satellites.value());
            if (gps.satellites.value() < 10) sats = "0" + sats;
            ggaMessage += sats + ",";
            
            // HDOP
            ggaMessage += String(gps.hdop.hdop(), 1) + ",";
            
            // Altitude
            ggaMessage += String(gps.altitude.meters() + altitudeCorrection, 1) + ",M,";
        } else {
            ggaMessage += "0000.0000,N,00000.0000,W,0,00,0.0,0.0,M,";
        }
        
        // Remaining fields - always the same
        ggaMessage += "0.0,M,,";

        // Calculate checksum
        uint8_t checksum = 0;
        for (size_t i = 1; i < ggaMessage.length(); i++) checksum ^= ggaMessage[i];
        ggaMessage += "*";
        if (checksum < 16) ggaMessage += "0";
        ggaMessage += String(checksum, HEX);
        ggaMessage += "\r\n"; // Explicit CRLF
        
        // Send GGA message and track statistics
        radio.print(ggaMessage);
        ggaMessagesSent++;
        
        // Track message quality
        if (!hasValidData) {
            nullMessages++;
        } else {
            convertedMessages++;
        }
        
        // Also send to USB if USB mode is enabled
        if (usbModeEnabled) {
            Serial.print(ggaMessage);
        }

        // Always send RMC message with consistent format
        String rmcMessage = "$GPRMC,";
        
        // Time field - always in same format
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
            rmcMessage += "000000.00,";
        }

        // Status and position fields - always in same format
        if (gps.location.isValid()) {
            rmcMessage += "A,"; // Status (A = valid)
            
            // Format latitude as DDMM.MMMM
            float lat = abs(gps.location.lat());
            int latDeg = (int)lat;
            float latMin = (lat - latDeg) * 60.0;
            String latDegStr = String(latDeg);
            if (latDeg < 10) latDegStr = "0" + latDegStr;
            String latMinStr = String(latMin, 4);
            while (latMinStr.length() < 7) latMinStr = "0" + latMinStr; // Ensure 4 decimal places
            rmcMessage += latDegStr + latMinStr + (gps.location.lat() < 0 ? ",S," : ",N,");

            // Format longitude as DDDMM.MMMM
            float lng = abs(gps.location.lng());
            int lngDeg = (int)lng;
            float lngMin = (lng - lngDeg) * 60.0;
            String lngDegStr = String(lngDeg);
            if (lngDeg < 100) lngDegStr = "0" + lngDegStr;
            if (lngDeg < 10) lngDegStr = "0" + lngDegStr;
            String lngMinStr = String(lngMin, 4);
            while (lngMinStr.length() < 7) lngMinStr = "0" + lngMinStr; // Ensure 4 decimal places
            rmcMessage += lngDegStr + lngMinStr + (gps.location.lng() < 0 ? ",W," : ",E,");

            // Speed
            rmcMessage += String(gps.speed.knots(), 1) + ",";
            
            // Course
            rmcMessage += String(gps.course.deg(), 1) + ",";
            
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
                rmcMessage += "010100,";
            }
        } else {
            rmcMessage += "V,0000.0000,N,00000.0000,W,0.0,0.0,010100,";
        }
        
        // Remaining fields - always the same
        rmcMessage += "0.0,E";

        // Calculate checksum
        checksum = 0;
        for (size_t i = 1; i < rmcMessage.length(); i++) checksum ^= rmcMessage[i];
        rmcMessage += "*";
        if (checksum < 16) rmcMessage += "0";
        rmcMessage += String(checksum, HEX);
        rmcMessage += "\r\n"; // Explicit CRLF
        
        // Send RMC message and track statistics
        radio.print(rmcMessage);
        rmcMessagesSent++;
        
        // Track message quality for RMC
        if (!hasValidData) {
            nullMessages++;
        } else {
            convertedMessages++;
        }
        
        // Also send to USB if USB mode is enabled
        if (usbModeEnabled) {
            Serial.print(rmcMessage);
        }

        lastSendTime = now;
    }

    // Check for non-standard messages (e.g., GMEA) and convert if needed
    while (radio.available()) {
        String line = radio.readStringUntil('\n');
        line.trim();
        if (line.startsWith("$GMEA")) {
            logMessage("Received non-standard GMEA: " + line);
            messageErrors++;
            
            // Also forward to USB if USB mode is enabled
            if (usbModeEnabled) {
                Serial.println(line);
            }
        }
    }

    // Report statistics every 10 seconds
    reportGPSStats();
}

// New method to forward NMEA data to USB port
void ICOM7100Configurator::forwardNMEAToUSB(TinyGPSPlus& gps, int altitudeCorrection) {
    static unsigned long lastSendTime = 0;
    const unsigned long FORWARD_INTERVAL = 500; // 2Hz

    unsigned long now = millis();
    if (now - lastSendTime >= FORWARD_INTERVAL) {
        // Always send GGA message
        String ggaMessage = "$GPGGA,";
        if (gps.time.isValid()) {
            // Format time as HHMMSS.SS
            ggaMessage += String(gps.time.hour(), DEC);
            if (gps.time.hour() < 10) ggaMessage += "0";
            ggaMessage += String(gps.time.minute(), DEC);
            if (gps.time.minute() < 10) ggaMessage += "0";
            ggaMessage += String(gps.time.second(), DEC);
            if (gps.time.second() < 10) ggaMessage += "0";
            ggaMessage += ".00,";
        } else {
            ggaMessage += ",,,";
        }

        if (gps.location.isValid()) {
            // Format latitude as DDMM.MMMM
            float lat = abs(gps.location.lat());
            int latDeg = (int)lat;
            float latMin = (lat - latDeg) * 60.0;
            ggaMessage += String(latDeg, DEC);
            if (latDeg < 10) ggaMessage += "0";
            ggaMessage += String(latMin, 4) + (gps.location.lat() < 0 ? "S," : "N,");

            // Format longitude as DDDMM.MMMM
            float lng = abs(gps.location.lng());
            int lngDeg = (int)lng;
            float lngMin = (lng - lngDeg) * 60.0;
            ggaMessage += String(lngDeg, DEC);
            if (lngDeg < 100) ggaMessage += "0";
            if (lngDeg < 10) ggaMessage += "0";
            ggaMessage += String(lngMin, 4) + (gps.location.lng() < 0 ? "W," : "E,");

            ggaMessage += "1,"; // Fix quality
            ggaMessage += String(gps.satellites.value()) + ",";
            ggaMessage += String(gps.hdop.hdop(), 1) + ",";
            ggaMessage += String(gps.altitude.meters() + altitudeCorrection, 1) + ",M,";
        } else {
            ggaMessage += ",,,,,,,";
        }
        ggaMessage += "0.0,M,,";

        // Calculate checksum
        uint8_t checksum = 0;
        for (size_t i = 1; i < ggaMessage.length(); i++) checksum ^= ggaMessage[i];
        ggaMessage += "*";
        if (checksum < 16) ggaMessage += "0";
        ggaMessage += String(checksum, HEX);
        ggaMessage += "\r\n"; // Explicit CRLF
        Serial.print(ggaMessage);  // Send to USB serial port

        // Always send RMC message
        String rmcMessage = "$GPRMC,";
        if (gps.time.isValid()) {
            // Format time as HHMMSS.SS
            rmcMessage += String(gps.time.hour(), DEC);
            if (gps.time.hour() < 10) rmcMessage += "0";
            rmcMessage += String(gps.time.minute(), DEC);
            if (gps.time.minute() < 10) rmcMessage += "0";
            rmcMessage += String(gps.time.second(), DEC);
            if (gps.time.second() < 10) rmcMessage += "0";
            rmcMessage += ".00,";
        } else {
            rmcMessage += ",,,";
        }

        if (gps.location.isValid()) {
            rmcMessage += "A,"; // Status
            // Format latitude as DDMM.MMMM
            float lat = abs(gps.location.lat());
            int latDeg = (int)lat;
            float latMin = (lat - latDeg) * 60.0;
            rmcMessage += String(latDeg, DEC);
            if (latDeg < 10) rmcMessage += "0";
            rmcMessage += String(latMin, 4) + (gps.location.lat() < 0 ? "S," : "N,");

            // Format longitude as DDDMM.MMMM
            float lng = abs(gps.location.lng());
            int lngDeg = (int)lng;
            float lngMin = (lng - lngDeg) * 60.0;
            rmcMessage += String(lngDeg, DEC);
            if (lngDeg < 100) rmcMessage += "0";
            if (lngDeg < 10) rmcMessage += "0";
            rmcMessage += String(lngMin, 4) + (gps.location.lng() < 0 ? "W," : "E,");

            rmcMessage += String(gps.speed.knots(), 1) + ",";
            rmcMessage += String(gps.course.deg(), 1) + ",";
            if (gps.date.isValid()) {
                // Format date as DDMMYY
                rmcMessage += String(gps.date.day(), DEC);
                if (gps.date.day() < 10) rmcMessage += "0";
                rmcMessage += String(gps.date.month(), DEC);
                if (gps.date.month() < 10) rmcMessage += "0";
                rmcMessage += String(gps.date.year() % 100, DEC);
            } else {
                rmcMessage += ",,,";
            }
        } else {
            rmcMessage += "V,,,,,,,,,";
        }
        rmcMessage += "0.0,E,";

        // Calculate checksum
        checksum = 0;
        for (size_t i = 1; i < rmcMessage.length(); i++) checksum ^= rmcMessage[i];
        rmcMessage += "*";
        if (checksum < 16) rmcMessage += "0";
        rmcMessage += String(checksum, HEX);
        rmcMessage += "\r\n"; // Explicit CRLF
        Serial.print(rmcMessage);  // Send to USB serial port

        lastSendTime = now;
    }
}

// USB Mode enabler
void ICOM7100Configurator::enableUSBMode(bool enable) {
    usbModeEnabled = enable;
    logMessage("USB mode " + String(enable ? "enabled" : "disabled"));
}

// USB Mode getter
bool ICOM7100Configurator::isUSBModeEnabled() const {
    return usbModeEnabled;
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
    
    // Also send to USB if USB mode is enabled
    if (usbModeEnabled) {
        Serial.write(statusCmd, sizeof(statusCmd));
    }
    
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
    setGPSBaudRate(4800);
    
    // Enable GPS display
    enableGPSDisplay();
    
    // Set default volume
    setVolume(50);
    
    // Set default squelch
    setSquelch(20);

    // Enable GPS-A functionality
    enableGPSA();
    
    // USB mode is disabled by default (set in constructor)
    logMessage("Radio initialized. USB mode: " + String(usbModeEnabled ? "ON" : "OFF"));
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