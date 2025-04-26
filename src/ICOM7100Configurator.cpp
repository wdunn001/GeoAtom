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
    
    // Send command with checksum
    radioSerial.print(cmd);
    if (checksum < 16) radioSerial.print("0");
    radioSerial.print(checksum, HEX);
    radioSerial.print("\r\n");
    
    lastCommandTime = millis();
}

// NMEA Forwarding
void ICOM7100Configurator::forwardNMEAToRadio(TinyGPSPlus& gps, int altitudeCorrection) {
    static unsigned long lastForwardTime = 0;
    const unsigned long FORWARD_INTERVAL = 1000; // Forward every second

    if (millis() - lastForwardTime >= FORWARD_INTERVAL) {
        if (gps.location.isValid()) {
            // Always send GGA (essential for position)
            String ggaMessage = "$GPGGA,";
            ggaMessage += String(gps.time.hour()) + String(gps.time.minute()) + String(gps.time.second()) + ".00,";
            ggaMessage += String(abs(gps.location.lat()), 4) + (gps.location.lat() < 0 ? "S," : "N,");
            ggaMessage += String(abs(gps.location.lng()), 4) + (gps.location.lng() < 0 ? "W," : "E,");
            ggaMessage += "1,"; // Fix quality
            ggaMessage += String(gps.satellites.value()) + ",";
            ggaMessage += String(gps.hdop.hdop(), 1) + ",";
            ggaMessage += String(gps.altitude.meters() + altitudeCorrection, 1) + ",M,";
            ggaMessage += "0.0,M,,"; // Geoid separation and age of diff

            // Calculate checksum
            uint8_t checksum = 0;
            for (size_t i = 1; i < ggaMessage.length(); i++) {
                checksum ^= ggaMessage[i];
            }
            ggaMessage += "*";
            if (checksum < 16) ggaMessage += "0";
            ggaMessage += String(checksum, HEX);
            ggaMessage += "\r\n";

            // Send GGA to Radio
            radioSerial.print(ggaMessage);

            // If we have valid course and speed, send RMC
            if (gps.course.isValid() && gps.speed.isValid()) {
                String rmcMessage = "$GPRMC,";
                rmcMessage += String(gps.time.hour()) + String(gps.time.minute()) + String(gps.time.second()) + ".00,";
                rmcMessage += "A,"; // Status (A=active)
                rmcMessage += String(abs(gps.location.lat()), 4) + (gps.location.lat() < 0 ? "S," : "N,");
                rmcMessage += String(abs(gps.location.lng()), 4) + (gps.location.lng() < 0 ? "W," : "E,");
                rmcMessage += String(gps.speed.knots(), 1) + ","; // Speed in knots
                rmcMessage += String(gps.course.deg(), 1) + ","; // Course in degrees
                rmcMessage += String(gps.date.day()) + String(gps.date.month()) + String(gps.date.year() % 100) + ","; // Date
                rmcMessage += "0.0,E,"; // Magnetic variation (not used)

                // Calculate checksum
                checksum = 0;
                for (size_t i = 1; i < rmcMessage.length(); i++) {
                    checksum ^= rmcMessage[i];
                }
                rmcMessage += "*";
                if (checksum < 16) rmcMessage += "0";
                rmcMessage += String(checksum, HEX);
                rmcMessage += "\r\n";

                // Send RMC to Radio
                radioSerial.print(rmcMessage);
            }

            // If we have valid satellite info, send GSV
            if (gps.satellites.isValid()) {
                String gsvMessage = "$GPGSV,";
                gsvMessage += "1,"; // Number of messages
                gsvMessage += "1,"; // Message number
                gsvMessage += String(gps.satellites.value()) + ","; // Number of satellites in view
                gsvMessage += "0,0,0,0,"; // Satellite info (not available in TinyGPS++)
                gsvMessage += "0,0,0,0,"; // More satellite info
                gsvMessage += "0,0,0,0"; // Final satellite info

                // Calculate checksum
                checksum = 0;
                for (size_t i = 1; i < gsvMessage.length(); i++) {
                    checksum ^= gsvMessage[i];
                }
                gsvMessage += "*";
                if (checksum < 16) gsvMessage += "0";
                gsvMessage += String(checksum, HEX);
                gsvMessage += "\r\n";

                // Send GSV to Radio
                radioSerial.print(gsvMessage);
            }

            // Always send GSA (DOP and active satellites)
            String gsaMessage = "$GPGSA,";
            gsaMessage += "A,"; // Auto selection
            gsaMessage += "3,"; // 3D fix
            gsaMessage += "0,0,0,0,0,0,0,0,0,0,0,0,0,"; // PRNs of satellites used
            gsaMessage += String(gps.hdop.hdop(), 1) + ","; // PDOP
            gsaMessage += String(gps.hdop.hdop(), 1) + ","; // HDOP
            gsaMessage += String(gps.hdop.hdop(), 1); // VDOP

            // Calculate checksum
            checksum = 0;
            for (size_t i = 1; i < gsaMessage.length(); i++) {
                checksum ^= gsaMessage[i];
            }
            gsaMessage += "*";
            if (checksum < 16) gsaMessage += "0";
            gsaMessage += String(checksum, HEX);
            gsaMessage += "\r\n";

            // Send GSA to Radio
            radioSerial.print(gsaMessage);
        }
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