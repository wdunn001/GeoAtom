#include "display_radio_status.h"
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "ICOM7100Configurator.h"
#include "display_manager.h"
#include "main_globals.h"

// Externs for globals used in these functions
extern Adafruit_SSD1306 display;
extern bool display_initialized;
extern UsbRadio usbRadio;
extern int radioFrequency;
extern String radioMode;
extern int radioPowerLevel;
extern DisplayMode currentDisplayMode;
extern void logMessage(const String& msg);

void displayRadioStatus() {
    if (!display_initialized) return;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    String title = "--- Radio Status ---";
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println(title);
    display.setCursor(0, 12);
    if (usbRadio.isConnected()) {
        display.print("ICOM 7100: Connected\n");
        display.print("Freq: ");
        display.println(radioFrequency);
        display.print("Mode: ");
        display.println(radioMode);
        display.print("Power: ");
        display.print(radioPowerLevel);
        display.println("W");
    } else {
        display.print("ICOM 7100: Not Connected\n");
        display.print("Connect USB to radio\n");
    }
    display.display();
}

void handleShortPressRadioStatus() {
    currentDisplayMode = DisplayMode::WIFI_STATUS;    
}

void handleLongPressRadioStatus() {
    // Switch to next screen
  currentDisplayMode = DisplayMode::RADIO_SETTINGS;
  logMessage("Display mode changed to: RADIO_SETTINGS");
} 