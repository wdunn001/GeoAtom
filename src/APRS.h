#ifndef APRS_H
#define APRS_H

#include <Arduino.h>
#include <WiFi.h>

class APRS {
public:
    APRS(const char* ssid, const char* password, 
         const char* callsign, const char* passcode,
         const char* server = "rotate.aprs2.net", 
         int port = 14580,
         int updateInterval = 300);
    
    bool begin();
    void update(float latitude, float longitude, float altitude, int heading, float speed);
    bool isConnected() const { return aprsConnected; }
    
private:
    const char* wifiSSID;
    const char* wifiPassword;
    const char* aprsCallsign;
    const char* aprsPasscode;
    const char* aprsServer;
    int aprsPort;
    int aprsInterval;
    
    bool aprsConnected;
    unsigned long lastAprsUpdate;
    WiFiClient aprsClient;
    
    void connectToAPRS();
    void sendPosition(float latitude, float longitude, float altitude, int heading, float speed);
    String createAPRSPacket(float latitude, float longitude, float altitude, int heading, float speed);
};

#endif // APRS_H 