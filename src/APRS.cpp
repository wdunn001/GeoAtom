#include "APRS.h"
#include <Arduino.h>
#include <WiFi.h>

APRS::APRS(const char* ssid, const char* password, 
           const char* callsign, const char* passcode,
           const char* server, int port, int updateInterval)
    : wifiSSID(ssid), wifiPassword(password),
      aprsCallsign(callsign), aprsPasscode(passcode),
      aprsServer(server), aprsPort(port), aprsInterval(updateInterval),
      aprsConnected(false), lastAprsUpdate(0) {
}

bool APRS::begin() {
    // Connect to WiFi
    WiFi.begin(wifiSSID, wifiPassword);
    
    // Wait for WiFi connection
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        attempts++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }
    
    // Connect to APRS server
    connectToAPRS();
    return aprsConnected;
}

void APRS::update(float latitude, float longitude, float altitude, int heading, float speed) {
    if (!aprsConnected) {
        connectToAPRS();
        return;
    }
    
    unsigned long currentTime = millis();
    if (currentTime - lastAprsUpdate >= aprsInterval * 1000) {
        sendPosition(latitude, longitude, altitude, heading, speed);
        lastAprsUpdate = currentTime;
    }
}

void APRS::connectToAPRS() {
    if (aprsClient.connect(aprsServer, aprsPort)) {
        // Send login string
        String login = "user " + String(aprsCallsign) + " pass " + String(aprsPasscode) + " vers GPS_Proj 1.0\r\n";
        aprsClient.print(login);
        aprsConnected = true;
    } else {
        aprsConnected = false;
    }
}

void APRS::sendPosition(float latitude, float longitude, float altitude, int heading, float speed) {
    if (!aprsConnected) return;
    
    String packet = createAPRSPacket(latitude, longitude, altitude, heading, speed);
    aprsClient.print(packet);
}

String APRS::createAPRSPacket(float latitude, float longitude, float altitude, int heading, float speed) {
    // Format: CALLSIGN>APRS,TCPIP*:!DDMM.MMNS/DDDMM.MMEW_CCC/SSS/A=AAAAAA
    String packet = String(aprsCallsign) + ">APRS,TCPIP*:";
    
    // Latitude
    int latDeg = abs((int)latitude);
    float latMin = (abs(latitude) - latDeg) * 60.0;
    packet += "!";
    packet += (latDeg < 10 ? "0" : "") + String(latDeg);
    packet += (latMin < 10 ? "0" : "") + String(latMin, 2);
    packet += (latitude >= 0 ? "N" : "S");
    
    // Longitude
    int lonDeg = abs((int)longitude);
    float lonMin = (abs(longitude) - lonDeg) * 60.0;
    packet += "/";
    packet += (lonDeg < 100 ? "0" : "") + String(lonDeg);
    packet += (lonMin < 10 ? "0" : "") + String(lonMin, 2);
    packet += (longitude >= 0 ? "E" : "W");
    
    // Course and Speed
    packet += "_";
    packet += (heading < 100 ? "0" : "") + String(heading);
    packet += "/";
    packet += (speed < 100 ? "0" : "") + String((int)speed);
    
    // Altitude
    packet += "/A=";
    packet += String((int)(altitude * 3.28084)); // Convert meters to feet
    
    packet += "\r\n";
    return packet;
} 