#ifndef ICOM7100_CONFIGURATOR_H
#define ICOM7100_CONFIGURATOR_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Forward declare the logMessage function
void logMessage(const String& msg);

class ICOM7100Configurator {
private:
    HardwareSerial& radio;
    unsigned long lastCommandTime;
    const unsigned long COMMAND_DELAY = 100; // 100ms delay between commands

    // GPS message statistics
    unsigned long ggaMessagesSent = 0;
    unsigned long rmcMessagesSent = 0;
    unsigned long messageErrors = 0;
    unsigned long nullMessages = 0;    // Messages with invalid data
    unsigned long convertedMessages = 0;    // Successfully converted GPS messages
    unsigned long lastStatsReportTime = 0;
    const unsigned long STATS_REPORT_INTERVAL = 10000; // 10 seconds

    // Helper function to send command with checksum
    void sendCommand(const String& cmd);
    void reportGPSStats(); // Method to report statistics
    
    // Helper methods for GPS message generation
    void generateBackupGSVMessages(TinyGPSPlus& gps, int satCount);
    void generateBackupGSAMessage(TinyGPSPlus& gps, int satCount);

public:
    // Constructor
    ICOM7100Configurator(HardwareSerial& serial);

    // Basic Commands
    void setFrequency(unsigned long freq);
    void setMode(const String& mode);
    void setPowerLevel(int level);

    // GPS Related Commands
    void enableGPSDisplay();
    void disableGPSDisplay();
    void setGPSBaudRate(int rate);
    void forwardNMEAToRadio(TinyGPSPlus& gps, int altitudeCorrection = 0);
    
    // Memory Operations
    void recallMemory(int channel);
    void storeMemory(int channel);

    // D-STAR Operations
    void setDStarCallSign(const String& callsign);
    void setDStarMessage(const String& message);

    // Scan Operations
    void startScan();
    void stopScan();

    // Voice Memory Operations
    void recordVoiceMemory(int channel);
    void playVoiceMemory(int channel);

    // Control Commands
    void setSquelch(int level);
    void setVolume(int level);

    // GPS-A Operations
    void enableGPSA();
    void disableGPSA();

    // Status Query
    bool queryStatus();

    // Initialize radio with default settings
    void initialize();
};

#endif // ICOM7100_CONFIGURATOR_H 