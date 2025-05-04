#include "display_radio_settings.h"
#include <Arduino.h>
#include "display_manager.h"
#include "ICOM7100Configurator.h"
#include "main_globals.h"
#include <U8g2lib.h>
extern UsbRadio usbRadio;
extern ICOM7100Configurator* radioConfig;

// Externs for globals used in these functions
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;
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

  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);

  // Center the title
  String title = "--- Radio Settings ---";
  int16_t x = (128 - display.getStrWidth(title.c_str())) / 2;
  display.drawStr(x, 10, title.c_str());

  // Define the settings area (leave space for command instructions at bottom)
  const int SETTINGS_START_Y = 20;
  const int SETTINGS_END_Y = 64 - 24; // Leave space for scroll indicator and commands
  const int SETTINGS_HEIGHT = SETTINGS_END_Y - SETTINGS_START_Y;
  const int ITEMS_PER_PAGE = 4; // Number of settings to show at once

  // Calculate which settings to show based on current index
  int startIndex = (radioSettingIndex / ITEMS_PER_PAGE) * ITEMS_PER_PAGE;
  int endIndex = min(startIndex + ITEMS_PER_PAGE, 8); // 8 total items (6 settings + 2 actions)

  // Display current settings with highlight for selected setting
  for (int i = startIndex; i < endIndex; i++) {
    int displayY = SETTINGS_START_Y + ((i - startIndex) * 10);
    String line;
    switch (i) {
      case 0:
        line = "Freq: " + String(radioFrequency / 1000000.0, 3) + "MHz";
        break;
      case 1:
        line = "Mode: " + radioMode;
        break;
      case 2:
        line = "Power: " + String(radioPowerLevel) + "W";
        break;
      case 3:
        line = "Mem: " + String(radioMemoryChannel);
        break;
      case 4:
        line = "D-STAR: ";
        line += (radioDStarCallSign.length() > 0) ? radioDStarCallSign : "OFF";
        break;
      case 5:
        line = "USB Mode: ";
        line += (radioUsbMode ? "ON" : "OFF");
        break;
      case 6:
        line = "Save & Exit";
        break;
      case 7:
        line = "Exit";
        break;
    }
    if (i == radioSettingIndex) {
      // Highlight selected line (invert area)
      display.setDrawColor(1);
      display.drawBox(0, displayY - 8, 128, 10);
      display.setDrawColor(0);
      display.drawStr(0, displayY, line.c_str());
      display.setDrawColor(1);
    } else {
      display.drawStr(0, displayY, line.c_str());
    }
  }

  // Show scroll indicator if there are more settings
  if (startIndex > 0 || endIndex < 8) {
    if (startIndex > 0) {
      display.drawStr(120, SETTINGS_START_Y, "^"); // Up arrow
    }
    if (endIndex < 8) {
      display.drawStr(120, SETTINGS_END_Y - 8, "v"); // Down arrow
    }
  }

  // Show navigation info at bottom
  String navInfo;
  if (isEditingSetting) {
    if (radioSettingIndex == 5) {
      navInfo = "Clk:Toggle  Hold:Save";
    } else {
      navInfo = "Clk: ^ Dbl: v Hold:Save";
    }
  } else {
    navInfo = "Clk:Next  Hold:Edit";
  }
  display.drawStr(0, 64 - 8, navInfo.c_str());

  display.sendBuffer();
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
    // If at end, wrap to RADIO_STATUS instead of WiFi status
    if (radioSettingIndex == 0) {
      currentDisplayMode = DisplayMode::RADIO_STATUS;
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
