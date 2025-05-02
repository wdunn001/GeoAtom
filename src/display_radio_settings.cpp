#include "display_radio_settings.h"
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "display_manager.h"
#include "ICOM7100Configurator.h"
#include "radio_web_server.h"
#include "main_globals.h"
extern UsbRadio usbRadio;
extern ICOM7100Configurator* radioConfig;

// Externs for globals used in these functions
extern Adafruit_SSD1306 display;
extern bool display_initialized;
extern int radioFrequency;
extern String radioMode;
extern int radioPowerLevel;
extern int radioMemoryChannel;
extern String radioDStarCallSign;
extern String radioDStarMessage;
extern bool radioUsbMode;
extern int radioSettingIndex;
extern bool isEditingSetting;
extern DisplayMode currentDisplayMode;
extern void logMessage(const String& msg);

void displayRadioSettings() {
  if (!display_initialized) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Center the title
  String title = "--- Radio Settings ---";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 0);
  display.println(title);

  // Define the settings area (leave space for command instructions at bottom)
  const int SETTINGS_START_Y = 12;
  const int SETTINGS_END_Y = SCREEN_HEIGHT - 24; // Leave space for scroll indicator and commands
  const int SETTINGS_HEIGHT = SETTINGS_END_Y - SETTINGS_START_Y;
  const int ITEMS_PER_PAGE = 4; // Number of settings to show at once

  // Calculate which settings to show based on current index
  int startIndex = (radioSettingIndex / ITEMS_PER_PAGE) * ITEMS_PER_PAGE;
  int endIndex = min(startIndex + ITEMS_PER_PAGE, 8); // 8 total items (6 settings + 2 actions)

  // Display current settings with highlight for selected setting
  for (int i = startIndex; i < endIndex; i++) {
    int displayY = SETTINGS_START_Y + ((i - startIndex) * 10);
    display.setCursor(0, displayY);
    
    if (i == radioSettingIndex) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    
    switch (i) {
      case 0:
        display.print("Freq: ");
        display.print(radioFrequency / 1000000.0, 3);
        display.print("MHz");
        break;
      case 1:
        display.print("Mode: ");
        display.print(radioMode);
        break;
      case 2:
        display.print("Power: ");
        display.print(radioPowerLevel);
        display.print("W");
        break;
      case 3:
        display.print("Mem: ");
        display.print(radioMemoryChannel);
        break;
      case 4:
        display.print("D-STAR: ");
        if (radioDStarCallSign.length() > 0) {
          display.print(radioDStarCallSign);
        } else {
          display.print("OFF");
        }
        break;
      case 5:
        display.print("USB Mode: ");
        display.print(radioUsbMode ? "ON" : "OFF");
        break;
      case 6:
        display.print("Save & Exit");
        break;
      case 7:
        display.print("Exit");
        break;
    }
  }

  // Show scroll indicator if there are more settings
  if (startIndex > 0 || endIndex < 8) {
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(SCREEN_WIDTH - 8, SETTINGS_START_Y);
    if (startIndex > 0) {
      display.print("^"); // Up arrow
    }
    display.setCursor(SCREEN_WIDTH - 8, SETTINGS_END_Y - 8);
    if (endIndex < 8) {
      display.print("v"); // Down arrow
    }
  }

  // Show navigation info at bottom
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, SCREEN_HEIGHT - 16);
  if (isEditingSetting) {
    if (radioSettingIndex == 5) {
      display.print("Clk:Toggle  Hold:Save");
    } else {
      display.print("Clk: ^ Dbl: v Hold:");
      // Draw save icon
      int x = display.getCursorX();
      int y = display.getCursorY();
      // Disk body
      display.fillRect(x, y, 8, 8, SSD1306_WHITE);
      // Disk label
      display.drawRect(x + 2, y + 2, 4, 4, SSD1306_BLACK);
      // Disk shutter
      display.drawRect(x - 1, y + 1, 2, 6, SSD1306_WHITE);
    }
  } else {
    display.print("Clk:Next  Hold:Edit");
  }

  display.display();
}

void handleShortPressRadioSettings() {
  if (isEditingSetting) {
    // Increment current setting
    switch (radioSettingIndex) {
      case 0: // Frequency
        radioFrequency += 1000000; // Increment by 1MHz
        if (radioFrequency > 148000000) radioFrequency = 144000000;
        break;
      case 1: // Mode
        if (radioMode == "FM") radioMode = "AM";
        else if (radioMode == "AM") radioMode = "LSB";
        else if (radioMode == "LSB") radioMode = "USB";
        else if (radioMode == "USB") radioMode = "CW";
        else if (radioMode == "CW") radioMode = "FM";
        break;
      case 2: // Power Level
        radioPowerLevel += 5;
        if (radioPowerLevel > 100) radioPowerLevel = 5;
        break;
      case 3: // Memory Channel
        radioMemoryChannel = (radioMemoryChannel + 1) % 100;
        break;
      case 4: // D-STAR
        if (radioDStarCallSign.length() == 0) {
          radioDStarCallSign = "N0CALL";
          radioDStarMessage = "Hello";
        } else {
          radioDStarCallSign = "";
          radioDStarMessage = "";
        }
        break;
      case 5: // USB Radio Mode
        radioUsbMode = !radioUsbMode;
        preferences.begin(PREF_NAMESPACE, false);
        preferences.putBool(KEY_RADIO_USB_MODE, radioUsbMode);
        preferences.end();
        logMessage(String("USB Radio Mode: ") + (radioUsbMode ? "ON" : "OFF"));
        break;
    }
  } else {
    // Select next setting/action
    radioSettingIndex = (radioSettingIndex + 1) % 8;
    // If at end, go to WiFi status
    if (radioSettingIndex == 0) {
      currentDisplayMode = DisplayMode::WIFI_STATUS;
      return;
    }
  }
}

void handleDoubleClickRadioSettings() {
  if (isEditingSetting) {
    // Decrement current setting
    switch (radioSettingIndex) {
      case 0: // Frequency
        radioFrequency -= 1000000; // Decrement by 1MHz
        if (radioFrequency < 144000000) radioFrequency = 148000000;
        break;
      case 1: // Mode
        if (radioMode == "FM") radioMode = "CW";
        else if (radioMode == "CW") radioMode = "USB";
        else if (radioMode == "USB") radioMode = "LSB";
        else if (radioMode == "LSB") radioMode = "AM";
        else if (radioMode == "AM") radioMode = "FM";
        break;
      case 2: // Power Level
        radioPowerLevel -= 5;
        if (radioPowerLevel < 5) radioPowerLevel = 100;
        break;
      case 3: // Memory Channel
        radioMemoryChannel = (radioMemoryChannel - 1 + 100) % 100;
        break;
      case 4: // D-STAR
        if (radioDStarCallSign.length() == 0) {
          radioDStarCallSign = "N0CALL";
          radioDStarMessage = "Hello";
        } else {
          radioDStarCallSign = "";
          radioDStarMessage = "";
        }
        break;
        break;
    }
    displayRadioSettings();
  }
  // No action in navigation mode
}

void handleLongPressRadioSettings() {
  if (isEditingSetting) {
    // Set current value and exit edit mode
    isEditingSetting = false;
    logMessage("Setting saved");
  } else {
    // Handle actions or enter edit mode
    if (radioSettingIndex == 6) { // Save & Exit
      currentDisplayMode = DisplayMode::LOG_DISPLAY;
      logMessage("Radio settings saved");
      return;
    } else if (radioSettingIndex == 7) { // Exit
      currentDisplayMode = DisplayMode::LOG_DISPLAY;
      logMessage("Radio settings not saved");
      return;
    } else {
      // Enter edit mode for current setting
      isEditingSetting = true;
      logMessage("Editing " + String(radioSettingIndex == 0 ? "Frequency" : 
                                   radioSettingIndex == 1 ? "Mode" :
                                   radioSettingIndex == 2 ? "Power" :
                                   radioSettingIndex == 3 ? "Memory" :
                                   radioSettingIndex == 4 ? "D-STAR" : "USB Mode"));
    }
  }
  displayRadioSettings();
} 

// Refactor radio settings application to use UsbRadio and ICOM7100Configurator
void applyRadioSettings() {
    if (radioUsbMode && usbRadio.isConnected()) {
        // Send settings over USB using ICOM7100Configurator's methods
        radioConfig->setFrequency(radioFrequency);
        radioConfig->setMode(radioMode);
        radioConfig->setPowerLevel(radioPowerLevel);
        // Add more as needed (memory, D-STAR, etc.)
        logMessage("Radio settings sent to ICOM 7100 over USB");
    } else if (radioConfig) {
        // Fallback to UART
        radioConfig->setFrequency(radioFrequency);
        radioConfig->setMode(radioMode);
        radioConfig->setPowerLevel(radioPowerLevel);
        logMessage("Radio settings sent to ICOM 7100 over UART");
    }
}
