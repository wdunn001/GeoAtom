#include "ICOM7100Configurator.h"

// Constructor implementation
ICOM7100Configurator::ICOM7100Configurator(HardwareSerial& serial) 
    : radio(serial), lastCommandTime(0) {
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
        radio.print(ggaMessage);

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
        radio.print(rmcMessage);

        lastSendTime = now;
    }

    // Check for non-standard messages (e.g., GMEA) and convert if needed
    while (radio.available()) {
        String line = radio.readStringUntil('\n');
        line.trim();
        if (line.startsWith("$GMEA")) {
            logMessage("Received non-standard GMEA: " + line);
        }
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