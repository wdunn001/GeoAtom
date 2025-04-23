#ifndef GPSConfigurator_H
#define GPSConfigurator_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Forward declaration if needed, or include relevant u-blox headers if available

class GPSConfigurator {
public:
    // Constructor
    GPSConfigurator(HardwareSerial& gps_serial);

    // --- Configuration Methods ---

    // Sets the UART baud rate for the GPS module.
    // Supported rates based on documentation: 4800, 9600, 19200, 38400, 57600, 115200
    // Returns true if a valid command was sent, false otherwise.
    bool setBaudRate(uint32_t baud);

    // Sets the navigation solution update rate.
    // Supported rates based on documentation: 1, 2, 4, 5, 8, 10 Hz
    // Returns true if a valid command was sent, false otherwise.
    bool setUpdateRateHz(uint8_t rate);

    // Enables or disables a specific NMEA message type.
    // msgClass: The NMEA message class (e.g., 0xF0 for NMEA Standard)
    // msgId: The specific NMEA message ID (e.g., 0x00 for GGA, 0x04 for RMC)
    // enable: true to enable, false to disable
    // Returns true if a valid command was sent, false otherwise.
    bool enableNmeaMessage(uint8_t msgClass, uint8_t msgId, bool enable);

    // Sets the dynamic platform model.
    // Models based on documentation: 0=Portable, 1=Fixed, 2=Stationary, 3=Pedestrian,
    // 4=Automotive, 5=Sea, 6=Airborne <1g, 7=Airborne <2g, 8=Airborne <4g
    // Returns true if a valid command was sent, false otherwise.
    bool setDynamicModel(uint8_t model);

    // --- Control Methods ---

    // Saves the current configuration to the module's non-volatile memory.
    // Returns true if the command was sent.
    bool saveConfiguration();

    // Loads the factory default configuration.
    // Returns true if the command was sent.
    bool loadFactoryDefaults();

    // Performs a cold start (resetting time, position, almanac, ephemeris).
    // Returns true if the command was sent.
    bool coldStart();

    // Performs a hot start (using existing time, position, almanac, ephemeris).
    // Returns true if the command was sent.
    bool hotStart();

    // Performs a controlled GPS software reset.
    // Returns true if the command was sent.
    bool reset();

private:
    HardwareSerial& _gps_serial; // Reference to the GPS serial port

    // Helper function to send a UBX command byte array
    void sendUbxCommand(const uint8_t* command, size_t length);

    // Helper to calculate UBX checksum (if needed, some commands might pre-calculate)
    void calculateChecksum(const uint8_t* payload, size_t length, uint8_t& ck_a, uint8_t& ck_b);
};

#endif // GPSConfigurator_H 