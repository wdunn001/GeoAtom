#include "display_manager.h"
#include <map>

// Externs for state flags and display mode
extern bool isSettingAltitudeCorrection;
extern bool isSelectingCalibrationMode;
extern bool isCalibrating;
extern bool isSettingDeclination;
extern bool isSettingInversion;
extern DisplayMode currentDisplayMode;

// Externs for all button handler functions
extern void handleShortPressGPSStatus();
extern void handleLongPressGPSStatus();
extern void handleShortPressCompassStatus();
extern void handleLongPressCompassStatus();
extern void handleDoubleClickCompassStatus();
extern void handleShortPressGraphicCompass();
extern void handleLongPressGraphicCompass();
extern void handleShortPressWorldMap();
extern void handleLongPressWorldMap();
extern void handleShortPressAltitudeCorrection();
extern void handleLongPressAltitudeCorrection();
extern void handleShortPressCalibrationModeSelection();
extern void handleLongPressCalibrationModeSelection();
extern void handleShortPressCalibrating();
extern void handleLongPressCalibrating();
extern void handleShortPressDeclination();
extern void handleLongPressDeclination();
extern void handleShortPressInversion();
extern void handleLongPressInversion();
extern void handleShortPressLogDisplay();
extern void handleLongPressLogDisplay();
extern void handleShortPressRadioSettings();
extern void handleLongPressRadioSettings();
extern void handleDoubleClickRadioSettings();
extern void handleShortPressRadioStatus();
extern void handleLongPressRadioStatus();
extern void handleShortPressWiFiStatus();
extern void handleLongPressWiFiStatus();
extern void handleShortPressBLEStatus();
extern void handleLongPressBLEStatus();

UIState currentUIState = UIState::GRAPHIC_COMPASS_SCREEN;

std::map<UIState, ButtonHandlers> buttonHandlerMap;

void setupButtonHandlers() {
  buttonHandlerMap[UIState::GPS_STATUS_SCREEN] = ButtonHandlers{handleShortPressGPSStatus, handleLongPressGPSStatus, nullptr};
  buttonHandlerMap[UIState::COMPASS_STATUS_SCREEN] = ButtonHandlers{handleShortPressCompassStatus, handleLongPressCompassStatus, handleDoubleClickCompassStatus};
  buttonHandlerMap[UIState::GRAPHIC_COMPASS_SCREEN] = ButtonHandlers{handleShortPressGraphicCompass, handleLongPressGraphicCompass, nullptr};
  buttonHandlerMap[UIState::WORLD_MAP_SCREEN] = ButtonHandlers{handleShortPressWorldMap, handleLongPressWorldMap, nullptr};
  buttonHandlerMap[UIState::ALTITUDE_CORRECTION_MODE] = ButtonHandlers{handleShortPressAltitudeCorrection, handleLongPressAltitudeCorrection, nullptr};
  buttonHandlerMap[UIState::CALIBRATION_MODE_SELECTION] = ButtonHandlers{handleShortPressCalibrationModeSelection, handleLongPressCalibrationModeSelection, nullptr};
  buttonHandlerMap[UIState::CALIBRATING_COMPASS] = ButtonHandlers{handleShortPressCalibrating, handleLongPressCalibrating, nullptr};
  buttonHandlerMap[UIState::SETTING_DECLINATION] = ButtonHandlers{handleShortPressDeclination, handleLongPressDeclination, nullptr};
  buttonHandlerMap[UIState::SETTING_INVERSION] = ButtonHandlers{handleShortPressInversion, handleLongPressInversion, nullptr};
  buttonHandlerMap[UIState::LOG_DISPLAY] = ButtonHandlers{handleShortPressLogDisplay, handleLongPressLogDisplay, nullptr};
  buttonHandlerMap[UIState::RADIO_SETTINGS_SCREEN] = ButtonHandlers{handleShortPressRadioSettings, handleLongPressRadioSettings, handleDoubleClickRadioSettings};
  buttonHandlerMap[UIState::WIFI_STATUS_SCREEN] = ButtonHandlers{handleShortPressWiFiStatus, handleLongPressWiFiStatus, nullptr};
  buttonHandlerMap[UIState::RADIO_STATUS_SCREEN] = ButtonHandlers{handleShortPressRadioStatus, handleLongPressRadioStatus, nullptr};
  buttonHandlerMap[UIState::BLE_STATUS_SCREEN] = ButtonHandlers{handleShortPressBLEStatus, handleLongPressBLEStatus, nullptr};
}

UIState determineCurrentUIState() {
  UIState detectedState;
  if (isSettingAltitudeCorrection) {
    detectedState = UIState::ALTITUDE_CORRECTION_MODE;
  }
  else if (isSelectingCalibrationMode) {
    detectedState = UIState::CALIBRATION_MODE_SELECTION;
  }
  else if (isCalibrating) {
    detectedState = UIState::CALIBRATING_COMPASS;
  }
  else if (isSettingDeclination) {
    detectedState = UIState::SETTING_DECLINATION;
  }
  else if (isSettingInversion) {
    detectedState = UIState::SETTING_INVERSION;
  }
  else {
    switch (currentDisplayMode) {
      case DisplayMode::GPS_STATUS:
        detectedState = UIState::GPS_STATUS_SCREEN;
        break;
      case DisplayMode::COMPASS_STATUS:
        detectedState = UIState::COMPASS_STATUS_SCREEN;
        break;
      case DisplayMode::GRAPHIC_COMPASS:
        detectedState = UIState::GRAPHIC_COMPASS_SCREEN;
        break;
      case DisplayMode::WORLD_MAP:
        detectedState = UIState::WORLD_MAP_SCREEN;
        break;
      case DisplayMode::LOG_DISPLAY:
        detectedState = UIState::LOG_DISPLAY;
        break;
      case DisplayMode::RADIO_SETTINGS:
        detectedState = UIState::RADIO_SETTINGS_SCREEN;
        break;
      case DisplayMode::WIFI_STATUS:
        detectedState = UIState::WIFI_STATUS_SCREEN;
        break;
      case DisplayMode::RADIO_STATUS:
        detectedState = UIState::RADIO_STATUS_SCREEN;
        break;
      case DisplayMode::BLE_STATUS:
        detectedState = UIState::BLE_STATUS_SCREEN;
        break;
      default:
        detectedState = UIState::GRAPHIC_COMPASS_SCREEN;
        break;
    }
  }
  return detectedState;
} 