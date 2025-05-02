#ifndef RADIO_WEB_SERVER_H
#define RADIO_WEB_SERVER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

struct WebRadioSettings {
    unsigned long frequency = 0;
    String mode;
    int powerLevel = 0;
    int memoryChannel = 0;
    String dstarCallSign;
    String dstarMessage;
    int squelch = 0;
    int volume = 0;
    bool gpsDisplay = false;
    int gpsBaudRate = 9600;
    bool gpsA = false;
    bool scan = false;
    int voiceMemChannel = 0;
    bool voiceMemRecord = false;
    bool voiceMemPlay = false;
};

extern WebRadioSettings webRadioSettings;

String radioHtml();
void setupRadioWebEndpoints(AsyncWebServer& server);

// Add WiFi status getter declarations
bool isWifiConnected();
String getWifiStatusMsg();
const char* getApSsid();
const char* getApPassword();

// Add getter for DNSServer instance
DNSServer& getDnsServer();

#endif // RADIO_WEB_SERVER_H 