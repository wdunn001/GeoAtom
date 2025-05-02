#include "radio_web_server.h"
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include "ICOM7100Configurator.h"
#include "main_globals.h"

// Extern declarations for globals/functions from main.cpp
extern void logMessage(const String& msg);
extern ICOM7100Configurator* radioConfig;
extern bool radioUsbMode;
extern Preferences preferences;
extern const char* PREF_NAMESPACE;
extern const char* KEY_RADIO_USB_MODE;

WebRadioSettings webRadioSettings;
// Global WiFi variables
const char* ap_ssid = "GeoAtom-Setup";
const char* ap_password = "";
AsyncWebServer server(80);
DNSServer dnsServer; // For captive portal functionality
Preferences wifiPrefs;
const char* WIFI_PREF_NAMESPACE = "wifi";
const char* WIFI_KEY_SSID = "ssid";
const char* WIFI_KEY_PASS = "pass";
String userWifiSSID = "";
String userWifiPass = "";
bool wifiConnected = false;
String wifiStatusMsg = "";

// Forward declarations for functions that use these variables
void tryConnectToUserWiFi();

void setupWiFiAndWeb();

// WiFi connection function
void tryConnectToUserWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(userWifiSSID.c_str(), userWifiPass.c_str());
    wifiStatusMsg = "Connecting to WiFi...";
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        wifiStatusMsg = "Connected! IP: " + WiFi.localIP().toString();
        // Disable AP for security
        WiFi.softAPdisconnect(true);
    } else {
        wifiConnected = false;
        wifiStatusMsg = "WiFi connect failed. Reverting to AP mode.";
        WiFi.disconnect();
        delay(500);
        WiFi.mode(WIFI_AP);
        WiFi.softAP(ap_ssid, ap_password);
    }
}

String settingsHtml() {
    String html = "<html><head><title>GeoAtom Settings</title>";
    html += "<script>function scanWifi(){window.open('/scan','scan','width=400,height=400');}</script>";
    html += "</head><body>";
    html += "<h2>GeoAtom Settings</h2>";
    html += "<a href='/radio'>Radio Control & Status</a><br>";
    if (!wifiStatusMsg.isEmpty()) html += "<p>" + wifiStatusMsg + "</p>";
    html += "<form method='POST' action='/settings'>";
    html += "<b>WiFi Setup</b><br>SSID: <input name='wifi_ssid' value='" + userWifiSSID + "'> <button type='button' onclick='scanWifi()'>Scan</button><br>";
    html += "Password: <input type='password' name='wifi_pass' value='" + userWifiPass + "'><br>";
    html += "</select><br>";
    html += "<input type='submit' value='Save'>";
    html += "</form></body></html>";
    return html;
}


void setupWiFiAndWeb() {
    // Load saved WiFi credentials
    wifiPrefs.begin(WIFI_PREF_NAMESPACE, true);
    userWifiSSID = wifiPrefs.getString(WIFI_KEY_SSID, "");
    userWifiPass = wifiPrefs.getString(WIFI_KEY_PASS, "");
    wifiPrefs.end();
    if (userWifiSSID.length() > 0) {
        tryConnectToUserWiFi();
    }
    if (!wifiConnected) {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(ap_ssid, ap_password);
        
        // Configure captive portal
        IPAddress apIP(192, 168, 4, 1);
        WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
        dnsServer.start(53, "*", apIP); // Capture all DNS requests
        logMessage("Captive portal started at " + apIP.toString());
        logMessage("You can access setup at http://geoatom.config/ while connected to GeoAtom-Setup WiFi");
    }
    
    // Main settings page
    server.on("/", HTTP_GET, [&](AsyncWebServerRequest *request){
        request->send(200, "text/html", settingsHtml());
    });
    
    // Setup page for geoatom.config
    server.on("/setup", HTTP_GET, [&](AsyncWebServerRequest *request){
        request->send(200, "text/html", settingsHtml());
    });
    
    // Captive portal detection - respond to special URLs used for captive portal detection
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request){
        request->redirect("/");
    });
    server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest *request){
        request->redirect("/");
    });
    server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request){
        request->redirect("/");
    });
    server.on("/connectivity-check", HTTP_GET, [](AsyncWebServerRequest *request){
        request->redirect("/");
    });
    server.on("/success.txt", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "success");
    });
    
    // Handle settings form submission
    server.on("/settings", HTTP_POST, [&](AsyncWebServerRequest *request){
        // WiFi credentials
        if (request->hasParam("wifi_ssid", true))
            userWifiSSID = request->getParam("wifi_ssid", true)->value();
        if (request->hasParam("wifi_pass", true))
            userWifiPass = request->getParam("wifi_pass", true)->value();
        // Save WiFi credentials
        wifiPrefs.begin(WIFI_PREF_NAMESPACE, false);
        wifiPrefs.putString(WIFI_KEY_SSID, userWifiSSID);
        wifiPrefs.putString(WIFI_KEY_PASS, userWifiPass);
        wifiPrefs.end();
        // Try to connect to user WiFi
        tryConnectToUserWiFi();
        // Other settings
        if (request->hasParam("radioFrequency", true))
            webRadioSettings.frequency = request->getParam("radioFrequency", true)->value().toInt();
        if (request->hasParam("radioMode", true))
            webRadioSettings.mode = request->getParam("radioMode", true)->value();
        if (request->hasParam("radioPowerLevel", true))
            webRadioSettings.powerLevel = request->getParam("radioPowerLevel", true)->value().toInt();
        if (request->hasParam("radioMemoryChannel", true))
            webRadioSettings.memoryChannel = request->getParam("radioMemoryChannel", true)->value().toInt();
        if (request->hasParam("radioDStarCallSign", true))
            webRadioSettings.dstarCallSign = request->getParam("radioDStarCallSign", true)->value();
        if (request->hasParam("radioDStarMessage", true))
            webRadioSettings.dstarMessage = request->getParam("radioDStarMessage", true)->value();
        webRadioSettings.gpsDisplay = request->hasParam("radioUsbMode", true);
        if (request->hasParam("gpsBaudRate", true))
            webRadioSettings.gpsBaudRate = request->getParam("gpsBaudRate", true)->value().toInt();
        webRadioSettings.gpsA = request->hasParam("compassInverted", true);
        if (request->hasParam("squelch", true))
            webRadioSettings.squelch = request->getParam("squelch", true)->value().toInt();
        if (request->hasParam("volume", true))
            webRadioSettings.volume = request->getParam("volume", true)->value().toInt();
        webRadioSettings.scan = request->hasParam("scan", true);
        if (request->hasParam("voiceMemChannel", true))
            webRadioSettings.voiceMemChannel = request->getParam("voiceMemChannel", true)->value().toInt();
        webRadioSettings.voiceMemRecord = request->hasParam("voiceMemRecord", true);
        webRadioSettings.voiceMemPlay = request->hasParam("voiceMemPlay", true);

        // Apply settings using ICOM7100Configurator
        if (radioConfig) {
            radioConfig->setFrequency(webRadioSettings.frequency);
            radioConfig->setMode(webRadioSettings.mode);
            radioConfig->setPowerLevel(webRadioSettings.powerLevel);
            radioConfig->setSquelch(webRadioSettings.squelch);
            radioConfig->setVolume(webRadioSettings.volume);
            if (webRadioSettings.gpsDisplay) radioConfig->enableGPSDisplay();
            else radioConfig->disableGPSDisplay();
            radioConfig->setGPSBaudRate(webRadioSettings.gpsBaudRate);
            if (webRadioSettings.gpsA) radioConfig->enableGPSA();
            else radioConfig->disableGPSA();
            if (request->hasParam("recallMemory", true))
                radioConfig->recallMemory(webRadioSettings.memoryChannel);
            if (request->hasParam("storeMemory", true))
                radioConfig->storeMemory(webRadioSettings.memoryChannel);
            radioConfig->setDStarCallSign(webRadioSettings.dstarCallSign);
            radioConfig->setDStarMessage(webRadioSettings.dstarMessage);
            if (request->hasParam("scanAction", true)) {
                String act = request->getParam("scanAction", true)->value();
                if (act == "start") radioConfig->startScan();
                else if (act == "stop") radioConfig->stopScan();
            }
            if (webRadioSettings.voiceMemRecord)
                radioConfig->recordVoiceMemory(webRadioSettings.voiceMemChannel);
            if (webRadioSettings.voiceMemPlay)
                radioConfig->playVoiceMemory(webRadioSettings.voiceMemChannel);
        }
        // Query status if requested
        if (request->hasParam("queryStatus", true) && radioConfig) {
            radioConfig->queryStatus();
        }
        request->send(200, "text/html", settingsHtml());
    });
    server.on("/scan", HTTP_GET, [&](AsyncWebServerRequest *request){
        int n = WiFi.scanNetworks();
        String html = "<html><body><h2>Available Networks</h2><ul>";
        for (int i = 0; i < n; ++i) {
            html += "<li onclick=\"window.opener.document.getElementsByName('wifi_ssid')[0].value='" + WiFi.SSID(i) + "';window.close();\">" + WiFi.SSID(i) + " (" + WiFi.RSSI(i) + ")" + (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? " (Open)" : "") + "</li>";
        }
        html += "</ul><button onclick=\"window.close()\">Close</button></body></html>";
        request->send(200, "text/html", html);
    });
    server.on("/radio", HTTP_GET, [&](AsyncWebServerRequest *request){
        request->send(200, "text/html", radioHtml());
    });
    server.on("/radio", HTTP_POST, [&](AsyncWebServerRequest *request){
        // Parse and apply radio settings
        if (request->hasParam("frequency", true))
            webRadioSettings.frequency = request->getParam("frequency", true)->value().toInt();
        if (request->hasParam("mode", true))
            webRadioSettings.mode = request->getParam("mode", true)->value();
        if (request->hasParam("powerLevel", true))
            webRadioSettings.powerLevel = request->getParam("powerLevel", true)->value().toInt();
        if (request->hasParam("memoryChannel", true))
            webRadioSettings.memoryChannel = request->getParam("memoryChannel", true)->value().toInt();
        if (request->hasParam("dstarCallSign", true))
            webRadioSettings.dstarCallSign = request->getParam("dstarCallSign", true)->value();
        if (request->hasParam("dstarMessage", true))
            webRadioSettings.dstarMessage = request->getParam("dstarMessage", true)->value();
        webRadioSettings.gpsDisplay = request->hasParam("gpsDisplay", true);
        if (request->hasParam("gpsBaudRate", true))
            webRadioSettings.gpsBaudRate = request->getParam("gpsBaudRate", true)->value().toInt();
        webRadioSettings.gpsA = request->hasParam("gpsA", true);
        if (request->hasParam("squelch", true))
            webRadioSettings.squelch = request->getParam("squelch", true)->value().toInt();
        if (request->hasParam("volume", true))
            webRadioSettings.volume = request->getParam("volume", true)->value().toInt();
        webRadioSettings.scan = request->hasParam("scan", true);
        if (request->hasParam("voiceMemChannel", true))
            webRadioSettings.voiceMemChannel = request->getParam("voiceMemChannel", true)->value().toInt();
        webRadioSettings.voiceMemRecord = request->hasParam("voiceMemRecord", true);
        webRadioSettings.voiceMemPlay = request->hasParam("voiceMemPlay", true);
        if (request->hasParam("radioUsbMode", true)) {
            radioUsbMode = true;
        } else {
            radioUsbMode = false;
        }
        preferences.begin(PREF_NAMESPACE, false);
        preferences.putBool(KEY_RADIO_USB_MODE, radioUsbMode);
        preferences.end();

        // Apply settings using ICOM7100Configurator
        if (radioConfig) {
            radioConfig->setFrequency(webRadioSettings.frequency);
            radioConfig->setMode(webRadioSettings.mode);
            radioConfig->setPowerLevel(webRadioSettings.powerLevel);
            radioConfig->setSquelch(webRadioSettings.squelch);
            radioConfig->setVolume(webRadioSettings.volume);
            if (webRadioSettings.gpsDisplay) radioConfig->enableGPSDisplay();
            else radioConfig->disableGPSDisplay();
            radioConfig->setGPSBaudRate(webRadioSettings.gpsBaudRate);
            if (webRadioSettings.gpsA) radioConfig->enableGPSA();
            else radioConfig->disableGPSA();
            if (request->hasParam("recallMemory", true))
                radioConfig->recallMemory(webRadioSettings.memoryChannel);
            if (request->hasParam("storeMemory", true))
                radioConfig->storeMemory(webRadioSettings.memoryChannel);
            radioConfig->setDStarCallSign(webRadioSettings.dstarCallSign);
            radioConfig->setDStarMessage(webRadioSettings.dstarMessage);
            if (request->hasParam("scanAction", true)) {
                String act = request->getParam("scanAction", true)->value();
                if (act == "start") radioConfig->startScan();
                else if (act == "stop") radioConfig->stopScan();
            }
            if (webRadioSettings.voiceMemRecord)
                radioConfig->recordVoiceMemory(webRadioSettings.voiceMemChannel);
            if (webRadioSettings.voiceMemPlay)
                radioConfig->playVoiceMemory(webRadioSettings.voiceMemChannel);
        }
        // Query status if requested
        if (request->hasParam("queryStatus", true) && radioConfig) {
            radioConfig->queryStatus();
        }
        request->send(200, "text/html", radioHtml());
    });
    server.onNotFound([](AsyncWebServerRequest *request){
        request->redirect("/");
    });
    server.begin();
    setupRadioWebEndpoints(server);
}
String radioHtml() {
    String html = "<html><head><title>ICOM-7100 Radio Control</title></head><body>";
    html += "<h2>ICOM-7100 Radio Control & Status</h2>";
    html += "<a href='/'>Back to Main Settings</a><br><br>";
    html += "<form method='POST' action='/radio'>";
    html += "<b>Radio Frequency (Hz):</b> <input name='frequency' value='" + String(webRadioSettings.frequency) + "'><br>";
    html += "<b>Mode:</b> <select name='mode'>";
    String modes[] = {"FM","AM","LSB","USB","CW"};
    for (auto m : modes) {
        html += "<option value='" + m + "'" + (webRadioSettings.mode==m?" selected":"") + ">" + m + "</option>";
    }
    html += "</select><br>";
    html += "<b>Power Level (W):</b> <input name='powerLevel' value='" + String(webRadioSettings.powerLevel) + "'><br>";
    html += "<b>Memory Channel:</b> <input name='memoryChannel' value='" + String(webRadioSettings.memoryChannel) + "'> ";
    html += "<button name='recallMemory' value='1'>Recall</button> <button name='storeMemory' value='1'>Store</button><br>";
    html += "<b>D-STAR CallSign:</b> <input name='dstarCallSign' value='" + webRadioSettings.dstarCallSign + "'><br>";
    html += "<b>D-STAR Message:</b> <input name='dstarMessage' value='" + webRadioSettings.dstarMessage + "'><br>";
    html += "<b>Squelch:</b> <input name='squelch' value='" + String(webRadioSettings.squelch) + "'><br>";
    html += "<b>Volume:</b> <input name='volume' value='" + String(webRadioSettings.volume) + "'><br>";
    html += "<b>GPS Display:</b> <input type='checkbox' name='gpsDisplay'" + String(webRadioSettings.gpsDisplay?" checked":"") + "><br>";
    html += "<b>GPS Baud Rate:</b> <input name='gpsBaudRate' value='" + String(webRadioSettings.gpsBaudRate) + "'><br>";
    html += "<b>GPS-A:</b> <input type='checkbox' name='gpsA'" + String(webRadioSettings.gpsA?" checked":"") + "><br>";
    html += "<b>Scan:</b> <input type='checkbox' name='scan'" + String(webRadioSettings.scan?" checked":"") + "> <button name='scanAction' value='start'>Start</button> <button name='scanAction' value='stop'>Stop</button><br>";
    html += "<b>Voice Memory Channel:</b> <input name='voiceMemChannel' value='" + String(webRadioSettings.voiceMemChannel) + "'> ";
    html += "<button name='voiceMemRecord' value='1'>Record</button> <button name='voiceMemPlay' value='1'>Play</button><br>";
    html += "<b>USB Radio Mode:</b> <input type='checkbox' name='radioUsbMode'";
    if (radioUsbMode) html += " checked";
    html += "><br>";
    html += "<input type='submit' value='Apply'>";
    html += "</form>";
    html += "<br><b>Current USB Radio Mode: </b>" + String(radioUsbMode ? "ON" : "OFF");
    html += "</body></html>";
    return html;
}

void setupRadioWebEndpoints(AsyncWebServer& server) {
    server.on("/radio", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", radioHtml());
    });
    server.on("/radio", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("frequency", true))
            webRadioSettings.frequency = request->getParam("frequency", true)->value().toInt();
        if (request->hasParam("mode", true))
            webRadioSettings.mode = request->getParam("mode", true)->value();
        if (request->hasParam("powerLevel", true))
            webRadioSettings.powerLevel = request->getParam("powerLevel", true)->value().toInt();
        if (request->hasParam("memoryChannel", true))
            webRadioSettings.memoryChannel = request->getParam("memoryChannel", true)->value().toInt();
        if (request->hasParam("dstarCallSign", true))
            webRadioSettings.dstarCallSign = request->getParam("dstarCallSign", true)->value();
        if (request->hasParam("dstarMessage", true))
            webRadioSettings.dstarMessage = request->getParam("dstarMessage", true)->value();
        webRadioSettings.gpsDisplay = request->hasParam("gpsDisplay", true);
        if (request->hasParam("gpsBaudRate", true))
            webRadioSettings.gpsBaudRate = request->getParam("gpsBaudRate", true)->value().toInt();
        webRadioSettings.gpsA = request->hasParam("gpsA", true);
        if (request->hasParam("squelch", true))
            webRadioSettings.squelch = request->getParam("squelch", true)->value().toInt();
        if (request->hasParam("volume", true))
            webRadioSettings.volume = request->getParam("volume", true)->value().toInt();
        webRadioSettings.scan = request->hasParam("scan", true);
        if (request->hasParam("voiceMemChannel", true))
            webRadioSettings.voiceMemChannel = request->getParam("voiceMemChannel", true)->value().toInt();
        webRadioSettings.voiceMemRecord = request->hasParam("voiceMemRecord", true);
        webRadioSettings.voiceMemPlay = request->hasParam("voiceMemPlay", true);
        if (request->hasParam("radioUsbMode", true)) {
            radioUsbMode = true;
        } else {
            radioUsbMode = false;
        }
        preferences.begin(PREF_NAMESPACE, false);
        preferences.putBool(KEY_RADIO_USB_MODE, radioUsbMode);
        preferences.end();
        // You may want to add logic to apply these settings to the radio here
        request->send(200, "text/html", radioHtml());
    });
}

bool isWifiConnected() {
    return wifiConnected;
}
String getWifiStatusMsg() {
    return wifiStatusMsg;
}
const char* getApSsid() {
    return ap_ssid;
}
const char* getApPassword() {
    return ap_password;
}

// Add getter for DNSServer instance
DNSServer& getDnsServer() {
    return dnsServer;
} 