#include "display_ble_status.h"
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "display_manager.h"
#include "main_globals.h"
#include "ble_service.h"

extern Adafruit_SSD1306 display;
extern bool display_initialized;
extern DisplayMode currentDisplayMode;
extern void logMessage(const String& msg);

void displayBLEStatus() {
    if (!display_initialized) return;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    String title = "--- BLE Status ---";
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println(title);
    display.setCursor(0, 12);
    display.print("Device: ");
    display.println(getBLEDeviceName());
    display.print("Status: ");
    display.println(isBLEConnected() ? "Connected" : "Not Connected");
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.print("Click: Compass");
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print("Hold: (none)");
    display.display();
}

void handleShortPressBLEStatus() {
    currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
    logMessage("Display mode changed to: GRAPHIC_COMPASS");
}

void handleLongPressBLEStatus() {
    // No action for now
} 