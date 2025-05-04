#include "display_wifi_status.h"
#include "display_manager.h"
#include <Arduino.h>
#include <WiFi.h>
#include "main_globals.h"

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;
extern bool display_initialized;
extern bool wifiSetupEnabled;
extern DisplayMode currentDisplayMode;
extern void logMessage(const String& msg);
extern bool isWifiConnected();
extern String getWifiStatusMsg();
extern const char* getApSsid();
extern const char* getApPassword();

void displayWiFiStatus() {
    if (!display_initialized) return;
    display.clearBuffer();
    setDisplayTitleStyle();
    String title = "--- WiFi Status ---";
    int w = display.getStrWidth(title.c_str());
    display.drawStr((SCREEN_WIDTH - w) / 2, 10, title.c_str());
    setDisplayDefaultStyle();
    int y = 24;
    if (WiFi.getMode() == WIFI_STA && isWifiConnected()) {
        String ssid = String("Mode: STA  SSID: ") + WiFi.SSID();
        display.drawStr(0, y, ssid.c_str());
        y += 10;
        String ip = String("IP: ") + WiFi.localIP().toString();
        display.drawStr(0, y, ip.c_str());
        y += 10;
    } else if (WiFi.getMode() & WIFI_AP) {
        String ap = String("Mode: AP  SSID: ") + getApSsid();
        display.drawStr(0, y, ap.c_str());
        y += 10;
        display.drawStr(0, y, "IP: 192.168.4.1");
        y += 10;
    } else {
        display.drawStr(0, y, "WiFi not active");
        y += 10;
    }
    display.drawStr(0, y, getWifiStatusMsg().c_str());
    display.drawStr(0, SCREEN_HEIGHT - 16, "Click: Error Log");
    if (wifiSetupEnabled) {
        display.drawStr(0, SCREEN_HEIGHT - 8, "Hold: Disable Setup");
    } else {
        display.drawStr(0, SCREEN_HEIGHT - 8, "Hold: Enable Setup");
    }
    display.sendBuffer();
}

void handleShortPressWiFiStatus() {
    currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
    logMessage("Display mode changed to: GRAPHIC_COMPASS");
}

void handleLongPressWiFiStatus() {
    wifiSetupEnabled = !wifiSetupEnabled;
    if (wifiSetupEnabled) {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(getApSsid(), getApPassword());
        logMessage("WiFi setup AP enabled. Connect to '" + String(getApSsid()) + "' to enter new credentials.");
    } else {
        WiFi.softAPdisconnect(true);
        logMessage("WiFi setup AP disabled.");
    }
} 