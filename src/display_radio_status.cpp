#include "display_radio_status.h"
#include <Arduino.h>
#include "ICOM7100Configurator.h"
#include "display_manager.h"
#include "main_globals.h"
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;

// Externs for globals used in these functions
// extern Adafruit_SSD1306 display;
extern bool display_initialized;
extern int radioFrequency;
extern String radioMode;
extern int radioPowerLevel;
extern DisplayMode currentDisplayMode;
extern void logMessage(const String& msg);

void displayRadioStatus() {
    if (!display_initialized) return;
    display.clearDisplay();
    setDisplayTitleStyle();
    String title = "--- Radio Status ---";
    display.setCursor((SCREEN_WIDTH - display.getStrWidth(title.c_str())) / 2, 0);
    display.println(title);
    setDisplayDefaultStyle();
    display.setCursor(0, 12);
    display.print("Freq: ");
    display.println(radioFrequency);
    display.print("Mode: ");
    display.println(radioMode);
    display.print("Power: ");
    display.print(radioPowerLevel);
    display.println("W");
    display.display();
}

void handleShortPressRadioStatus() {
    // Removed RADIO_SETTINGS mode
}

void handleLongPressRadioStatus() {
    // Removed RADIO_SETTINGS mode
} 