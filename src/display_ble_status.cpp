// BLE display code disabled for test
/*
#include "display_ble_status.h"
#include <Arduino.h>
#include "display_manager.h"
#include "main_globals.h"
#include "ble_service.h"

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;
extern bool display_initialized;
extern DisplayMode currentDisplayMode;
extern void logMessage(const String& msg);

void displayBLEStatus() {
    if (!display_initialized) return;
    display.clearBuffer();
    display.setFont(u8g2_font_ncenB08_tr);
    display.setDrawColor(1);
    String title = "--- BLE Status ---";
    int w = display.getStrWidth(title.c_str());
    display.drawStr((SCREEN_WIDTH - w) / 2, 10, title.c_str());
    int y = 24;
    String dev = String("Device: ") + getBLEDeviceName();
    display.drawStr(0, y, dev.c_str());
    y += 10;
    String stat = String("Status: ") + (isBLEConnected() ? "Connected" : "Not Connected");
    display.drawStr(0, y, stat.c_str());
    y += 16;
    display.drawStr(0, SCREEN_HEIGHT - 16, "Click: Compass");
    display.drawStr(0, SCREEN_HEIGHT - 8, "Hold: (none)");
    display.sendBuffer();
}

void handleShortPressBLEStatus() {
    currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
    logMessage("Display mode changed to: GRAPHIC_COMPASS");
}

void handleLongPressBLEStatus() {
    // No action for now
}
*/ 