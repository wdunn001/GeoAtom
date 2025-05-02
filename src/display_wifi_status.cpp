#include "display_wifi_status.h"
#include "display_manager.h"
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include "main_globals.h"

// Externs for globals used in these functions
extern Adafruit_SSD1306 display;
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
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    String title = "--- WiFi Status ---";
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println(title);
    display.setCursor(0, 12);
    if (WiFi.getMode() == WIFI_STA && isWifiConnected()) {
        display.print("Mode: STA\nSSID: ");
        display.println(WiFi.SSID());
        display.print("IP: ");
        display.println(WiFi.localIP().toString());
    } else if (WiFi.getMode() & WIFI_AP) {
        display.print("Mode: AP\nSSID: ");
        display.println(getApSsid());
        display.print("IP: 192.168.4.1\n");
    } else {
        display.println("WiFi not active");
    }
    display.setCursor(0, 36);
    display.println(getWifiStatusMsg());
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.print("Click: Error Log");
    display.setCursor(0, SCREEN_HEIGHT - 8);
    if (wifiSetupEnabled) {
        display.print("Hold: Disable Setup");
    } else {
        display.print("Hold: Enable Setup");
    }
    display.display();
}

void handleShortPressWiFiStatus() {
    currentDisplayMode = DisplayMode::LOG_DISPLAY;
    logMessage("Display mode changed to: LOG_DISPLAY");
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