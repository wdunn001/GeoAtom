#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

// Shared display mode enum for all display modules
enum class DisplayMode {
    GPS_STATUS,
    COMPASS_STATUS,
    GRAPHIC_COMPASS,
    WORLD_MAP,
    LOG_DISPLAY,
    RADIO_SETTINGS,
    WIFI_STATUS,
    RADIO_STATUS,
    BLE_STATUS
};

// Shared UI state enum for button handler logic
enum class UIState {
    GPS_STATUS_SCREEN,
    COMPASS_STATUS_SCREEN,
    GRAPHIC_COMPASS_SCREEN,
    WORLD_MAP_SCREEN,
    RADIO_SETTINGS_SCREEN,
    WIFI_STATUS_SCREEN,
    RADIO_STATUS_SCREEN,
    ALTITUDE_CORRECTION_MODE,
    CALIBRATION_MODE_SELECTION,
    CALIBRATING_COMPASS,
    SETTING_DECLINATION,
    SETTING_INVERSION,
    LOG_DISPLAY,
    BLE_STATUS_SCREEN
};

// Define a type for button handler functions for clarity
typedef void (*ButtonHandlerFunc)();

// Define a struct to hold both short and long press handlers
struct ButtonHandlers {
    ButtonHandlerFunc shortPressHandler;
    ButtonHandlerFunc longPressHandler;
    ButtonHandlerFunc doubleClickHandler;
};

void setupButtonHandlers(); 

void displayBLEStatus();

#endif // DISPLAY_MANAGER_H 