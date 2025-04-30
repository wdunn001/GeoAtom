#include "ICOM7100Configurator.h"

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
            float lat = abs(gps.location.lat());
            int latDeg = (int)lat;
            float latMin = (lat - latDeg) * 60.0;
            String latDegStr = String(latDeg);
            if (latDeg < 10) latDegStr = "0" + latDegStr;
            
            char latMinStr[10];
            sprintf(latMinStr, "%07.4f", latMin); // Ensure 4 decimal places with leading zeros
            ggaMessage += latDegStr + latMinStr + "," + (gps.location.lat() < 0 ? "S," : "N,");

            // Format longitude as DDDMM.MMMM
            float lng = abs(gps.location.lng());
            int lngDeg = (int)lng;
            float lngMin = (lng - lngDeg) * 60.0;
            String lngDegStr = String(lngDeg);
            if (lngDeg < 100) lngDegStr = "0" + lngDegStr;
            if (lngDeg < 10) lngDegStr = "0" + lngDegStr;
            
            char lngMinStr[10];
            sprintf(lngMinStr, "%07.4f", lngMin); // Ensure 4 decimal places with leading zeros
            ggaMessage += lngDegStr + lngMinStr + "," + (gps.location.lng() < 0 ? "W," : "E,");

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
            float lat = abs(gps.location.lat());
            int latDeg = (int)lat;
            float latMin = (lat - latDeg) * 60.0;
            String latDegStr = String(latDeg);
            if (latDeg < 10) latDegStr = "0" + latDegStr;
            
            char latMinStr[10];
            sprintf(latMinStr, "%07.4f", latMin); // Ensure 4 decimal places with leading zeros
            rmcMessage += latDegStr + latMinStr + "," + (gps.location.lat() < 0 ? "S," : "N,");

            // Format longitude as DDDMM.MMMM
            float lng = abs(gps.location.lng());
            int lngDeg = (int)lng;
            float lngMin = (lng - lngDeg) * 60.0;
            String lngDegStr = String(lngDeg);
            if (lngDeg < 100) lngDegStr = "0" + lngDegStr;
            if (lngDeg < 10) lngDegStr = "0" + lngDegStr;
            
            char lngMinStr[10];
            sprintf(lngMinStr, "%07.4f", lngMin); // Ensure 4 decimal places with leading zeros
            rmcMessage += lngDegStr + lngMinStr + "," + (gps.location.lng() < 0 ? "W," : "E,");

            // Speed
            if (gps.speed.isValid()) {
            rmcMessage += String(gps.speed.knots(), 1) + ",";
            } else {
                rmcMessage += "0.0,"; // Zero speed if invalid
            }
            
            // Course
            if (gps.course.isValid()) {
            rmcMessage += String(gps.course.deg(), 1) + ",";
            } else {
                rmcMessage += "0.0,"; // Zero course if invalid
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

// Helper method to generate backup GSV messages with dummy data
void ICOM7100Configurator::generateBackupGSVMessages(TinyGPSPlus& gps, int satCount) {
    // Use the provided satCount parameter passed from the caller
    // This ensures consistency across all messages
    
    // Calculate how many messages are needed (max 4 satellites per message)
    int numMessages = (satCount + 3) / 4; // Ceiling division
    
    // Generate GSV messages - each can contain up to 4 satellites
    for (int msgNum = 1; msgNum <= numMessages; msgNum++) {
        String gsvMessage = "$GPGSV," + String(numMessages) + "," + String(msgNum) + "," + String(satCount);
        
        // Calculate satellite range for this message
        int startSat = (msgNum - 1) * 4;
        int endSat = min(startSat + 3, satCount - 1);
        
        // Add 4 satellites (or fewer for the last message)
        for (int i = startSat; i <= endSat; i++) {
            // PRN ranging from 1-32 for GPS
            int prn = i + 1;
            if (prn > 32) prn = prn - 32; // Wrap around if we exceed 32
            
            // Need to include actual values for elevation, azimuth, and SNR
            // Using simple patterns to distribute satellites around sky view
            int elevation = 15 + (i * 5) % 75; // 15-90 degrees elevation
            int azimuth = (i * 30) % 360;      // 0-359 degrees azimuth
            int snr = 20 + (i * 3) % 30;       // 20-50 dB SNR (reasonable values)
            
            // Format: ,PRN,elevation,azimuth,SNR
            gsvMessage += "," + String(prn);
            
            // Only include 2 digits for values (NMEA standard)
            gsvMessage += "," + String(elevation);
            gsvMessage += "," + String(azimuth);
            gsvMessage += "," + String(snr);
        }
        
        // Calculate checksum
        uint8_t checksum = 0;
        for (size_t i = 1; i < gsvMessage.length(); i++) {
            checksum ^= gsvMessage[i];
        }
        
        gsvMessage += "*";
        if (checksum < 16) gsvMessage += "0";
        gsvMessage += String(checksum, HEX);
        gsvMessage += "\r\n";
        
        // Send GSV message
        radio.print(gsvMessage);
    }
    
    logMessage("Generated " + String(numMessages) + " backup GSV messages with sat count: " + String(satCount));
}

// Helper method to generate backup GSA message with dummy data
void ICOM7100Configurator::generateBackupGSAMessage(TinyGPSPlus& gps, int satCount) {
    // Use the provided satCount parameter passed from the caller
    // This ensures consistency across all messages
    
    // Add a GSA message - GPS DOP and active satellites
    // A=Auto 2D/3D selection, M=Manual, 1=No fix, 2=2D fix, 3=3D fix
    String gsaMessage = "$GPGSA,A,3,"; // A=Auto, 3=3D fix
    
    // Include up to 12 PRNs in GSA message
    // These should match the PRNs we're using in the GSV messages
    int maxSatsInGSA = min(satCount, 12); // GSA message can include up to 12 satellites
    
    // Add PRN numbers for the satellites used in fix
    for (int i = 0; i < 12; i++) {
        if (i < maxSatsInGSA) {
            // Use same PRN calculation as in GSV messages
            int prn = i + 1;
            if (prn > 32) prn = prn - 32; // Wrap around if we exceed 32
            
            // Format PRN with leading zero if needed
            if (prn < 10) {
                gsaMessage += "0";
            }
            gsaMessage += String(prn) + ",";
    } else {
            // Empty field for unused satellite slots
            gsaMessage += ",";
    }
    }
    
    // Calculate realistic DOP values based on number of satellites
    // More satellites = better DOP values
    float pdopBase = 5.0;
    float hdopBase = 2.5;
    float vdopBase = 4.0;
    
    float satFactor = 1.0 - (min(satCount, 12) / 24.0); // 0.5 to 0.83 depending on sats
    float pdop = pdopBase * satFactor;
    float hdop = hdopBase * satFactor;
    float vdop = vdopBase * satFactor;
    
    // Format to one decimal place
    char dopStr[30];
    sprintf(dopStr, "%.1f,%.1f,%.1f", pdop, hdop, vdop);
    gsaMessage += dopStr;
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 1; i < gsaMessage.length(); i++) {
        checksum ^= gsaMessage[i];
    }
    
    gsaMessage += "*";
    if (checksum < 16) gsaMessage += "0";
    gsaMessage += String(checksum, HEX);
    gsaMessage += "\r\n";
    
    // Send GSA message
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