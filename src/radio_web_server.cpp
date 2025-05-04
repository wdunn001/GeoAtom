#include "radio_web_server.h"
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include "ICOM7100Configurator.h"
#include "main_globals.h"
#include "yuma_http_service.h"

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
WebServer server(80);
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

void onWiFiEvent(WiFiEvent_t event) {
    if (event == SYSTEM_EVENT_STA_GOT_IP) {
        if (!isYumaFileCurrent()) {
            if (ensureYumaAlmanacCurrent()) {
                logMessage("Yuma almanac updated after WiFi connect.");
            } else {
                logMessage("Failed to update Yuma almanac after WiFi connect (no internet or endpoint unreachable).");
            }
        } else {
            logMessage("Yuma almanac already current after WiFi connect.");
        }
    }
}

void setupWiFiAndWeb() {
    // Register WiFi event handler
    WiFi.onEvent(onWiFiEvent);
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
    server.on("/", HTTP_GET, [](){
        server.send(200, "text/html", settingsHtml());
    });
    
    // Setup page for geoatom.config
    server.on("/setup", HTTP_GET, [](){
        server.send(200, "text/html", settingsHtml());
    });
    
    // Captive portal detection - respond to special URLs used for captive portal detection
    server.on("/generate_204", HTTP_GET, [](){
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "");
    });
    server.on("/fwlink", HTTP_GET, [](){
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "");
    });
    server.on("/hotspot-detect.html", HTTP_GET, [](){
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "");
    });
    server.on("/connectivity-check", HTTP_GET, [](){
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "");
    });
    server.on("/success.txt", HTTP_GET, [](){
        server.send(200, "text/plain", "success");
    });
    
    // Handle settings form submission
    server.on("/settings", HTTP_POST, [&server](){
        // WiFi credentials
        if (server.hasArg("wifi_ssid"))
            userWifiSSID = server.arg("wifi_ssid");
        if (server.hasArg("wifi_pass"))
            userWifiPass = server.arg("wifi_pass");
        // Save WiFi credentials
        wifiPrefs.begin(WIFI_PREF_NAMESPACE, false);
        wifiPrefs.putString(WIFI_KEY_SSID, userWifiSSID);
        wifiPrefs.putString(WIFI_KEY_PASS, userWifiPass);
        wifiPrefs.end();
        // Try to connect to user WiFi
        tryConnectToUserWiFi();
        // Other settings
        if (server.hasArg("radioFrequency"))
            webRadioSettings.frequency = server.arg("radioFrequency").toInt();
        if (server.hasArg("radioMode"))
            webRadioSettings.mode = server.arg("radioMode");
        if (server.hasArg("radioPowerLevel"))
            webRadioSettings.powerLevel = server.arg("radioPowerLevel").toInt();
        if (server.hasArg("radioMemoryChannel"))
            webRadioSettings.memoryChannel = server.arg("radioMemoryChannel").toInt();
        if (server.hasArg("radioDStarCallSign"))
            webRadioSettings.dstarCallSign = server.arg("radioDStarCallSign");
        if (server.hasArg("radioDStarMessage"))
            webRadioSettings.dstarMessage = server.arg("radioDStarMessage");
        webRadioSettings.gpsDisplay = server.hasArg("radioUsbMode");
        if (server.hasArg("gpsBaudRate"))
            webRadioSettings.gpsBaudRate = server.arg("gpsBaudRate").toInt();
        webRadioSettings.gpsA = server.hasArg("compassInverted");
        if (server.hasArg("squelch"))
            webRadioSettings.squelch = server.arg("squelch").toInt();
        if (server.hasArg("volume"))
            webRadioSettings.volume = server.arg("volume").toInt();
        webRadioSettings.scan = server.hasArg("scan");
        if (server.hasArg("voiceMemChannel"))
            webRadioSettings.voiceMemChannel = server.arg("voiceMemChannel").toInt();
        webRadioSettings.voiceMemRecord = server.hasArg("voiceMemRecord");
        webRadioSettings.voiceMemPlay = server.hasArg("voiceMemPlay");

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
            if (server.hasArg("recallMemory"))
                radioConfig->recallMemory(webRadioSettings.memoryChannel);
            if (server.hasArg("storeMemory"))
                radioConfig->storeMemory(webRadioSettings.memoryChannel);
            radioConfig->setDStarCallSign(webRadioSettings.dstarCallSign);
            radioConfig->setDStarMessage(webRadioSettings.dstarMessage);
            if (server.hasArg("scanAction")) {
                String act = server.arg("scanAction");
                if (act == "start") radioConfig->startScan();
                else if (act == "stop") radioConfig->stopScan();
            }
            if (webRadioSettings.voiceMemRecord)
                radioConfig->recordVoiceMemory(webRadioSettings.voiceMemChannel);
            if (webRadioSettings.voiceMemPlay)
                radioConfig->playVoiceMemory(webRadioSettings.voiceMemChannel);
        }
        // Query status if requested
        if (server.hasArg("queryStatus") && radioConfig) {
            radioConfig->queryStatus();
        }
        server.send(200, "text/html", settingsHtml());
    });
    server.on("/scan", HTTP_GET, [&server](){
        int n = WiFi.scanNetworks();
        String html = "<html><body><h2>Available Networks</h2><ul>";
        for (int i = 0; i < n; ++i) {
            html += "<li onclick=\"window.opener.document.getElementsByName('wifi_ssid')[0].value='" + WiFi.SSID(i) + "';window.close();\">" + WiFi.SSID(i) + " (" + WiFi.RSSI(i) + ")" + (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? " (Open)" : "") + "</li>";
        }
        html += "</ul><button onclick=\"window.close()\">Close</button></body></html>";
        server.send(200, "text/html", html);
    });
    server.onNotFound([&server](){
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "");
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
    html += "<input type='submit' value='Apply'>";
    html += "</form>";
    html += "<br><b>Current USB Radio Mode: </b>" + String(radioUsbMode ? "ON" : "OFF");
    html += "</body></html>";
    return html;
}

void setupRadioWebEndpoints(WebServer& server) {
    server.on("/radio", HTTP_GET, [&server](){
        server.send(200, "text/html", radioHtml());
    });
    server.on("/radio", HTTP_POST, [&server](){
        if (server.hasArg("frequency"))
            webRadioSettings.frequency = server.arg("frequency").toInt();
        if (server.hasArg("mode"))
            webRadioSettings.mode = server.arg("mode");
        if (server.hasArg("powerLevel"))
            webRadioSettings.powerLevel = server.arg("powerLevel").toInt();
        if (server.hasArg("memoryChannel"))
            webRadioSettings.memoryChannel = server.arg("memoryChannel").toInt();
        if (server.hasArg("dstarCallSign"))
            webRadioSettings.dstarCallSign = server.arg("dstarCallSign");
        if (server.hasArg("dstarMessage"))
            webRadioSettings.dstarMessage = server.arg("dstarMessage");
        webRadioSettings.gpsDisplay = server.hasArg("gpsDisplay");
        if (server.hasArg("gpsBaudRate"))
            webRadioSettings.gpsBaudRate = server.arg("gpsBaudRate").toInt();
        webRadioSettings.gpsA = server.hasArg("gpsA");
        if (server.hasArg("squelch"))
            webRadioSettings.squelch = server.arg("squelch").toInt();
        if (server.hasArg("volume"))
            webRadioSettings.volume = server.arg("volume").toInt();
        webRadioSettings.scan = server.hasArg("scan");
        if (server.hasArg("voiceMemChannel"))
            webRadioSettings.voiceMemChannel = server.arg("voiceMemChannel").toInt();
        webRadioSettings.voiceMemRecord = server.hasArg("voiceMemRecord");
        webRadioSettings.voiceMemPlay = server.hasArg("voiceMemPlay");
        // You may want to add logic to apply these settings to the radio here
        server.send(200, "text/html", radioHtml());
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