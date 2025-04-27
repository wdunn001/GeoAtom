#ifndef ICOM7100_CONFIGURATOR_H
#define ICOM7100_CONFIGURATOR_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Forward declare the logMessage function
void logMessage(const String& msg);

class ICOM7100Configurator {
private:
    HardwareSerial& radioSerial;
    unsigned long lastCommandTime;
    const unsigned long COMMAND_DELAY = 100; // 100ms delay between commands

    // Helper function to send command with checksum
    void sendCommand(const String& cmd);

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

    // Initialize radio with default settings
    void initialize();
};

#endif // ICOM7100_CONFIGURATOR_H 