#ifndef ICOM7100_CONFIGURATOR_H
#define ICOM7100_CONFIGURATOR_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <vector>
#include <map>

// Forward declare the logMessage function
void logMessage(const String& msg);

// Structure to represent a satellite
struct SatelliteInfo {
    int prn;                // Satellite PRN number
    int elevation;          // Elevation in degrees
    int azimuth;            // Azimuth in degrees
    int snr;                // Signal-to-Noise Ratio (0 if not available)
    unsigned long lastSeen; // Last time this satellite was seen (millis)
    bool used;              // Whether this satellite is used in position fix
    
    SatelliteInfo() : prn(0), elevation(0), azimuth(0), snr(0), lastSeen(0), used(false) {}
    
    SatelliteInfo(int _prn, int _elev, int _az, int _snr) 
        : prn(_prn), elevation(_elev), azimuth(_az), snr(_snr), lastSeen(millis()), used(false) {}
};

// Class for managing satellite data aggregation
class SatelliteDataManager {
private:
    std::map<int, SatelliteInfo> satellites;      // Map of PRN to satellite info
    std::vector<int> usedPRNs;                    // PRNs used in position solution
    unsigned long lastFullUpdate;                 // Last time a full GSV set was processed
    unsigned long lastTransmitTime;               // Last time data was transmitted to radio
    unsigned long SAT_TIMEOUT;                    // Timeout for satellites (set in constructor)
    unsigned long UPDATE_INTERVAL;                // Interval between full updates (set in constructor)
    
public:
    SatelliteDataManager();
    
    // Process a GSV message and update satellite database
    bool processGSVMessage(const String& gsvMessage);
    
    // Process a GSA message to update which satellites are used
    bool processGSAMessage(const String& gsaMessage);
    
    // Generate optimized GSV messages for transmission
    std::vector<String> generateGSVMessages();
    
    // Generate optimized GSA message for transmission
    String generateGSAMessage();
    
    // Get stats about current satellites
    int getTotalSatellites() const;
    int getVisibleSatellites() const;
    int getUsedSatellitesCount() const;  // Get count of satellites used for position fix
    std::vector<int> getUsedSatellites(); // Get list of PRNs used for position fix
    
    // Check if a satellite is prioritized for display (used for fix or good signal)
    bool isPrioritizedForDisplay(int prn) const;
    
    // Should we transmit new data?
    bool shouldTransmit();
    
    // Mark that we've transmitted data
    void markTransmitted();
    
    // Cleanup expired satellites
    void cleanup();
    
    // Force transmission of satellite data
    void forceTransmit() { lastTransmitTime = 0; }
};

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
    const unsigned long STATS_REPORT_INTERVAL = 10000; // 10 seconds between reports
    
    // Constants for radio communications and backup data
    const unsigned long RADIO_SEND_INTERVAL = 1000; // Send to radio at 1 Hz
    const unsigned long MSG_TIMEOUT = 5000; // 5 seconds without data triggers backup data
    const unsigned long BACKUP_DATA_TIMEOUT = 10000; // 10 seconds timeout for backup data
    const unsigned long BACKUP_REFRESH_INTERVAL = 5000; // 5 seconds interval for refreshing backup data
    
    // Backup data flag and variables
    bool _usingBackupData = false;
    std::vector<String> _backupGSVs;
    String _lastGGA;
    String _lastRMC;
    
    // Satellite data manager
    SatelliteDataManager satelliteManager;
    
    // Helper functions
    String nullNMEA(const String& prefix);
    String generateFixedGSAMessage(); // New method to create a GSA message with satellites used for fix
    
public:
    ICOM7100Configurator(HardwareSerial& serial);
    
    void sendCommand(const String& cmd);
    void reportGPSStats(int satCount, unsigned long charsPerSecond); // Method to report statistics with satellite count and chars/s
    
    // GPS to Radio data forwarding
    void generateBackupGSVMessages(TinyGPSPlus& gps, int satCount);
    void generateBackupGSAMessage(TinyGPSPlus& gps, int satCount);
    
    // Radio frequency and mode control
    void setFrequency(unsigned long freq);
    void setMode(const String& mode);
    void setPowerLevel(int level);
    
    // GPS display control
    void enableGPSDisplay();
    void disableGPSDisplay();
    void setGPSBaudRate(int rate);
    void forwardNMEAToRadio(TinyGPSPlus& gps, int altitudeCorrection = 0, unsigned long charsPerSecond = 0);
    
    // Query if using backup data
    bool isUsingBackupData() { return _usingBackupData; }
    
    // Get last generated NMEA messages
    String getLastGeneratedGGA() { return _lastGGA; }
    String getLastGeneratedRMC() { return _lastRMC; }
    
    // Memory channel control
    void recallMemory(int channel);
    void storeMemory(int channel);
    
    // D-STAR support
    void setDStarCallSign(const String& callsign);
    void setDStarMessage(const String& message);
    
    // Scanning control
    void startScan();
    void stopScan();
    
    // Voice memory control
    void recordVoiceMemory(int channel);
    void playVoiceMemory(int channel);
    
    // Audio control
    void setSquelch(int level);
    void setVolume(int level);
    
    // GPS NMEA data forwarding settings
    void enableGPSA();
    void disableGPSA();
    
    // Status query
    bool queryStatus();
    
    // Initialization
    void initialize();
    void reportGPSStats();
};

#endif // ICOM7100_CONFIGURATOR_H 