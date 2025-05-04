#include "display_radio_status.h"
#include <Arduino.h>
#include "ICOM7100Configurator.h"
#include "display_manager.h"
#include "main_globals.h"

// Externs for globals used in these functions
// extern Adafruit_SSD1306 display;
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
    display.setFont(u8g2_font_ncenB08_tr);
    display.setDrawColor(1);
    String title = "--- Radio Status ---";
    display.setCursor((SCREEN_WIDTH - display.getStrWidth(title.c_str())) / 2, 0);
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
    currentDisplayMode = DisplayMode::RADIO_SETTINGS;
    logMessage("Display mode changed to: RADIO_SETTINGS");
}

void handleLongPressRadioStatus() {
    // Switch to next screen
  currentDisplayMode = DisplayMode::RADIO_SETTINGS;
  logMessage("Display mode changed to: RADIO_SETTINGS");
} 