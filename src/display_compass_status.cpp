#include "CompassInterface.h"
#include <Preferences.h>
#include "display_compass_status.h"
#include <Arduino.h>
#include "display_manager.h"
#include "main_globals.h"
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;

extern bool display_initialized;
extern CompassInterface* activeCompass;
extern bool isSelectingCalibrationMode;
extern int calibrationModeIndex;
extern const char* CALIBRATION_MODE_NAMES[];
extern bool isCalibrating;
extern int calibrationPoints;
extern int calibrationStep;
extern int* calX;
extern int* calY;
extern Preferences preferences;
extern const char* PREF_NAMESPACE;
extern const char* KEY_CAL_VALID;
extern const char* KEY_X_OFFSET;
extern const char* KEY_Y_OFFSET;
extern const char* KEY_Z_OFFSET;
extern const char* KEY_X_SCALE;
extern const char* KEY_Y_SCALE;
extern const char* KEY_Z_SCALE;
extern const char* KEY_HMC_CAL_VALID;
extern const char* KEY_HMC_X_OFFSET;
extern const char* KEY_HMC_Y_OFFSET;
extern const char* KEY_HMC_Z_OFFSET;
extern const char* KEY_HMC_X_SCALE;
extern const char* KEY_HMC_Y_SCALE;
extern const char* KEY_HMC_Z_SCALE;
extern bool isSettingDeclination;
extern float currentDeclination;
extern HMC5883LCompassImpl hmcCompass;
extern QMC5883LCompassImpl qmcCompass;
extern bool isSettingInversion;
extern bool compassInverted;
extern void logMessage(const String& msg);
extern DisplayMode currentDisplayMode;
extern void resetCompass();
extern const char* KEY_DECLINATION;
extern const char* KEY_COMPASS_INVERTED;

void displayCompassStatusOnOLED() {
  if (!display_initialized) return;

  // Add static variable to track last update time
  static unsigned long lastUpdateTime = 0;
  const unsigned long UPDATE_INTERVAL = 100; // Update every 100ms (10Hz)

  // Check if enough time has passed since last update
  if (millis() - lastUpdateTime < UPDATE_INTERVAL) {
    return; // Skip this update
  }
  lastUpdateTime = millis();

  display.clearBuffer();
  setDisplayDefaultStyle();

  // Only show calibration mode selection and multi-point calibration for QMC5883L
  if (isSelectingCalibrationMode && activeCompass == &qmcCompass) {
    display.setCursor(0, 0);
    display.println("Select Cal Mode:");
    display.println("---------------");
    for (int i = 0; i < NUM_CALIBRATION_MODES; i++) {
      if (i == calibrationModeIndex) {
        display.print("> ");
      } else {
        display.print("  ");
      }
      display.println(CALIBRATION_MODE_NAMES[i]);
    }
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.println("Click: Next Option");
    display.println("Hold: Select Option");
  }
  else if (isCalibrating && activeCompass == &qmcCompass) {
    display.setCursor(0, 0);
    display.println("-- Calibrating --");
    display.setCursor(0, 10);
    display.print("Mode: ");
    display.print(calibrationPoints);
    display.println("-point");
    display.setCursor(0, 26);
    String directions = "";
    if (calibrationPoints == 4) {
      switch (calibrationStep) {
        case 1: directions = "Point NORTH, Click"; break;
        case 2: directions = "Point EAST, Click"; break;
        case 3: directions = "Point SOUTH, Click"; break;
        case 4: directions = "Point WEST, Click"; break;
      }
    } 
    else if (calibrationPoints == 8) {
      switch (calibrationStep) {
        case 1: directions = "Point NORTH, Click"; break;
        case 2: directions = "Point SOUTH, Click"; break;
        case 3: directions = "Point EAST, Click"; break;
        case 4: directions = "Point WEST, Click"; break;
        case 5: directions = "Point NE, Click"; break;
        case 6: directions = "Point SE, Click"; break;
        case 7: directions = "Point SW, Click"; break;
        case 8: directions = "Point NW, Click"; break;
      }
    } 
    else {
      directions = "Point to " + String(calibrationStep * 22.5) + " deg";
    }
    display.println(directions);
    display.setCursor(0, 46);
    display.print("Progress: ");
    display.print(calibrationStep - 1);
    display.print("/");
    display.println(calibrationPoints);
    display.setCursor(0, 56);
    display.println("(Hold Btn to Cancel)");
  }
  else if (isSettingDeclination && activeCompass == &hmcCompass) {
    display.setCursor(0, 0);
    display.println("- Set Declination -");
    display.setCursor(0, 16);
    display.println("Point device to TRUE");
    display.setCursor(0, 26);
    display.println("NORTH, then click");
    display.setCursor(0, 40);
    display.print("Current: ");
    display.print(activeCompass->getAzimuth());
    display.println(" deg");
    display.setCursor(0, 56);
    display.println("Hold: Cancel");
  }
  else if (isSettingInversion) {
    display.setCursor(0, 0);
    display.println("Invert Compass?");
    display.println("---------------");
    
    int centerX = SCREEN_WIDTH / 2;
    int centerY = 30;
    int radius = 15;
    
    if (activeCompass != nullptr) {
      int heading = activeCompass->getAzimuth();
      if (compassInverted) {
        heading = (heading + 180) % 360;
      }
      
      float angleRad = radians(270 - heading);
      int endX = centerX + radius * cos(angleRad);
      int endY = centerY + radius * sin(angleRad);
      
      display.drawCircle(centerX, centerY, radius);
      display.drawLine(centerX, centerY, endX, endY);
      display.drawDisc(centerX, centerY, 2);
      
      display.setCursor(centerX - 2, centerY - radius - 6);
      display.print("N");
      display.setCursor(centerX - 2, centerY + radius);
      display.print("S");
      display.setCursor(centerX + radius, centerY - 2);
      display.print("E");
      display.setCursor(centerX - radius - 5, centerY - 2);
      display.print("W");
      
      display.setCursor(68, 14);
      display.print("Hdg:");
      display.print(heading);
    }
    
    display.setCursor(0, SCREEN_HEIGHT - 18);
    display.println("Click: Normal");
    display.setCursor(70, SCREEN_HEIGHT - 18);
    display.println("Hold: Invert");
    
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print("Current: ");
    display.print(compassInverted ? "Inverted" : "Normal");
  }
  else {
    // Normal Compass Status Screen
    String title = "-- Compass Status --";
    setDisplayTitleStyle();
    int titleWidth = display.getStrWidth(title.c_str());
    display.setCursor((SCREEN_WIDTH - titleWidth) / 2, 0);
    display.println(title);
    setDisplayDefaultStyle();
    
    if (activeCompass != nullptr) {
      // Display compass type on first line
      display.setCursor(0, 12);
      display.print(activeCompass->getSensorName());
      
      // Show declination if using HMC5883L on same line
      if (activeCompass == &hmcCompass) {
        float declDegrees = currentDeclination * 180.0 / PI;
        display.print(" Decl:");
        display.print(declDegrees, 1);
      }
      
      // Display heading on next line
      activeCompass->read();
      int displayHeading = activeCompass->getAzimuth();
      if (compassInverted) {
        displayHeading = (displayHeading + 180) % 360;
      }
      
      // Move heading and direction to next line
      display.setCursor(0, 24);
      display.print("Az ");
      display.print(displayHeading);
      display.print(" deg ");
      
      char dirArray[4] = {' ', ' ', ' ', '\0'};
      activeCompass->getDirection(dirArray, displayHeading);
      display.print(dirArray);
      
      // Raw sensor data with proper spacing
      display.setCursor(0, 36);
      display.print("X: ");
      display.println(activeCompass->getX());
      
      display.setCursor(0, 44);
      display.print("Y: ");
      display.println(activeCompass->getY());
      
      display.setCursor(0, 52);
      display.print("Z: ");
      display.println(activeCompass->getZ());
      
      // Show inversion status at bottom
      if (compassInverted) {
        String invStr = "[Inverted]";
        int invStrWidth = display.getStrWidth(invStr.c_str());
        display.setCursor(SCREEN_WIDTH - invStrWidth, SCREEN_HEIGHT - 8);
        display.print(invStr);
      }
    } else {
      display.setCursor(0, 24);
      display.println("No compass detected");
    }
    
    // Add calibration prompt in bottom right if compass is active
    if (activeCompass != nullptr) {
      String calibPrompt = "";
      if (activeCompass == &qmcCompass) {
        calibPrompt = "Cal: Hold";
      } else if (activeCompass == &hmcCompass) {
        calibPrompt = "Decl: Hold";
      }
      
      // Right align the calibration prompt
      int calibPromptWidth = display.getStrWidth(calibPrompt.c_str());
      display.setCursor(SCREEN_WIDTH - calibPromptWidth, SCREEN_HEIGHT - 8);
      display.print(calibPrompt);
    }
  }

  display.sendBuffer();
}

void handleShortPressCompassStatus() {
    currentDisplayMode = DisplayMode::WIFI_STATUS;
    logMessage("Display mode changed to: WIFI_STATUS");
}

void handleLongPressCompassStatus() {
    if (activeCompass == &qmcCompass) {
        isSelectingCalibrationMode = true;
        calibrationModeIndex = 1; // Default to 8-point
        logMessage("Entering calibration mode selection...");
    } else if (activeCompass == &hmcCompass) {
        isSettingDeclination = true;
        logMessage("Entering declination setting mode...");
    }
} 
// Calibration Mode Selection
void handleShortPressCalibrationModeSelection() {
  // Short press in calibration mode selection: Rotate through options
  calibrationModeIndex = (calibrationModeIndex + 1) % NUM_CALIBRATION_MODES;
  logMessage("Calibration mode: " + String(CALIBRATION_MODE_NAMES[calibrationModeIndex]));
}

void handleLongPressCalibrationModeSelection() {
  // Long press in calibration mode selection: Select current mode
  logMessage("DEBUG: Starting calibration selection handler...");
  
  if (calibrationModeIndex == NUM_CALIBRATION_MODES - 1) {
    // Selected "Cancel" option
    isSelectingCalibrationMode = false;
    logMessage("Calibration cancelled");
  } else {
    // Selected a valid calibration mode
    calibrationPoints = CALIBRATION_MODE_POINTS[calibrationModeIndex];
    isSelectingCalibrationMode = false;
    isCalibrating = true;
    calibrationStep = 1;
    logMessage("DEBUG: Set isCalibrating=" + String(isCalibrating) + ", calibrationPoints=" + String(calibrationPoints) + ", step=" + String(calibrationStep));
    logMessage("Starting " + String(calibrationPoints) + "-point calibration...");
  }
}

// Calibrating Compass
void handleShortPressCalibrating() {
  // Short press while calibrating: Capture calibration point
  if (calibrationStep > 0 && calibrationStep <= calibrationPoints) {
    logMessage("Capturing calibration point " + String(calibrationStep) + " of " + String(calibrationPoints));
    
    if (activeCompass) {
      activeCompass->read();
      calX[calibrationStep - 1] = activeCompass->getX();
      calY[calibrationStep - 1] = activeCompass->getY();
      calibrationStep++;

      // If last point captured, calculate and apply calibration
      if (calibrationStep > calibrationPoints) {
        logMessage("Calculating " + String(calibrationPoints) + "-point calibration...");
        
        if (activeCompass->calculateCalibration(calX, calY)) {
          logMessage("Calibration calculated and applied successfully.");
          
          // Calculate noise and interference characteristics from calibration data
          float x_variance = 0.0f;
          float y_variance = 0.0f;
          float x_mean = 0.0f;
          float y_mean = 0.0f;
          
          // Calculate mean
          for (int i = 0; i < calibrationPoints; i++) {
            x_mean += calX[i];
            y_mean += calY[i];
          }
          x_mean /= calibrationPoints;
          y_mean /= calibrationPoints;
          
          // Calculate variance
          for (int i = 0; i < calibrationPoints; i++) {
            x_variance += (calX[i] - x_mean) * (calX[i] - x_mean);
            y_variance += (calY[i] - y_mean) * (calY[i] - y_mean);
          }
          x_variance /= calibrationPoints;
          y_variance /= calibrationPoints;
          

          
          // Save to NVM based on compass type
          preferences.begin(PREF_NAMESPACE, false);
          
          if (activeCompass == &qmcCompass) {
            // For QMC compass
            // Set calibration as valid
            preferences.putBool(KEY_CAL_VALID, true);
            
            // Since we don't have direct access to the internal calibration values,
            // we can calculate them from the min/max of our collected points
            float x_min = calX[0], x_max = calX[0];
            float y_min = calY[0], y_max = calY[0];
            
            for (int i = 1; i < calibrationPoints; i++) {
              if (calX[i] < x_min) x_min = calX[i];
              if (calX[i] > x_max) x_max = calX[i];
              if (calY[i] < y_min) y_min = calY[i];
              if (calY[i] > y_max) y_max = calY[i];
            }
            
            float x_offset = (x_max + x_min) / 2.0f;
            float y_offset = (y_max + y_min) / 2.0f;
            float z_offset = 0.0f; // We don't use Z for calibration
            
            float x_delta = (x_max - x_min) / 2.0f;
            float y_delta = (y_max - y_min) / 2.0f;
            float avg_delta = (x_delta + y_delta) / 2.0f;
            
            float x_scale = (x_delta > 0) ? avg_delta / x_delta : 1.0f;
            float y_scale = (y_delta > 0) ? avg_delta / y_delta : 1.0f;
            float z_scale = 1.0f;
            
            // Save calculated values
            preferences.putFloat(KEY_X_OFFSET, x_offset);
            preferences.putFloat(KEY_Y_OFFSET, y_offset);
            preferences.putFloat(KEY_Z_OFFSET, z_offset);
            preferences.putFloat(KEY_X_SCALE, x_scale);
            preferences.putFloat(KEY_Y_SCALE, y_scale);
            preferences.putFloat(KEY_Z_SCALE, z_scale);
            

            
            logMessage("QMC5883L calibration and interference settings saved.");
          } 
          else if (activeCompass == &hmcCompass) {
            // For HMC compass
            // Same calculations as for QMC to ensure consistency
            preferences.putBool(KEY_HMC_CAL_VALID, true);
            
            float x_min = calX[0], x_max = calX[0];
            float y_min = calY[0], y_max = calY[0];
            
            for (int i = 1; i < calibrationPoints; i++) {
              if (calX[i] < x_min) x_min = calX[i];
              if (calX[i] > x_max) x_max = calX[i];
              if (calY[i] < y_min) y_min = calY[i];
              if (calY[i] > y_max) y_max = calY[i];
            }
            
            float x_offset = (x_max + x_min) / 2.0f;
            float y_offset = (y_max + y_min) / 2.0f;
            float z_offset = 0.0f;
            
            float x_delta = (x_max - x_min) / 2.0f;
            float y_delta = (y_max - y_min) / 2.0f;
            float avg_delta = (x_delta + y_delta) / 2.0f;
            
            float x_scale = (x_delta > 0) ? avg_delta / x_delta : 1.0f;
            float y_scale = (y_delta > 0) ? avg_delta / y_delta : 1.0f;
            float z_scale = 1.0f;
            
            // Save calculated values
            preferences.putFloat(KEY_HMC_X_OFFSET, x_offset);
            preferences.putFloat(KEY_HMC_Y_OFFSET, y_offset);
            preferences.putFloat(KEY_HMC_Z_OFFSET, z_offset);
            preferences.putFloat(KEY_HMC_X_SCALE, x_scale);
            preferences.putFloat(KEY_HMC_Y_SCALE, y_scale);
            preferences.putFloat(KEY_HMC_Z_SCALE, z_scale);
            
            logMessage("HMC5883L calibration and interference settings saved.");
          }
          preferences.end();
          
          // Exit calibration mode
          isCalibrating = false;
          calibrationStep = 0;
          
          // Move to inversion setting
          isSettingInversion = true;
        
          logMessage("Applied interference settings: ");
 
        } else {
          logMessage("Calibration calculation failed. Try again.");
          isCalibrating = false;
          calibrationStep = 0;
        }
      }
    }
  }
}

void handleLongPressCalibrating() {
  // Long press while calibrating: Cancel calibration
  isCalibrating = false;
  calibrationStep = 0;
  logMessage("Calibration cancelled");
}

// Setting Declination
void handleShortPressDeclination() {
  // Short press in declination setting: Set declination
  if (activeCompass == &hmcCompass) {
    activeCompass->read();
    int magneticHeading = activeCompass->getAzimuth(); // Read the current magnetic heading

    // Calculate the declination angle
    // Declination = True North (0) - Magnetic Heading read when pointing True North
    float declinationDegrees = -magneticHeading;

    // Normalize to +/- 180 degrees
    while (declinationDegrees > 180.0f) declinationDegrees -= 360.0f;
    while (declinationDegrees <= -180.0f) declinationDegrees += 360.0f;

    // Convert to radians for storage and setting
    currentDeclination = radians(declinationDegrees);
  
    // Apply declination immediately
    hmcCompass.setDeclination(currentDeclination);
    
    // Save to NVM
    preferences.begin(PREF_NAMESPACE, false);
    preferences.putFloat(KEY_DECLINATION, currentDeclination);
    preferences.end();
    
    logMessage("Declination set to: " + String(declinationDegrees, 1) + " degrees");
    logMessage("True north should now be at compass heading: 0 deg");
    
    // Make note about calibration and declination working together
    logMessage("HMC5883L now has calibration + declination applied");
    
    // Exit declination setting
    isSettingDeclination = false;
    
    // Move to inversion setting
    isSettingInversion = true;
  }
}

void handleLongPressDeclination() {
  // Long press in declination setting: Cancel
  isSettingDeclination = false;
  logMessage("Declination setting cancelled");
}
// Setting Inversion
void handleShortPressInversion() {
  // Short press in inversion setting: Set normal mode
  compassInverted = false;
  
  // Save to NVM
  preferences.begin(PREF_NAMESPACE, false);
  preferences.putBool(KEY_COMPASS_INVERTED, false);
  preferences.end();
  
  logMessage("Compass display set to normal (non-inverted) mode");
  isSettingInversion = false;
  
  // If we're using HMC5883L and haven't set declination yet, allow it now
  if (activeCompass == &hmcCompass && !isSettingDeclination) {
    isSettingDeclination = true;
    logMessage("Now set declination for HMC5883L...");
    logMessage("Point to TRUE NORTH, then click the button.");
  }
}
void handleLongPressInversion() {
  // Long press in inversion setting: Set inverted mode
  compassInverted = true;
  
  // Save to NVM
  preferences.begin(PREF_NAMESPACE, false);
  preferences.putBool(KEY_COMPASS_INVERTED, true);
  preferences.end();
  
  logMessage("Compass display set to inverted mode");
  isSettingInversion = false;
}

void handleDoubleClickCompassStatus() {
  // Reset compass on double click
  resetCompass();
  logMessage("Compass reset triggered by double click");
}

void resetCompass() {
  // Clear all compass preferences
  preferences.begin(PREF_NAMESPACE, false);
  preferences.clear();
  preferences.end();
  
  // Reset compass state variables
  compassInverted = false;
  currentDeclination = 0.0f;
  isCalibrating = false;
  isSettingDeclination = false;
  isSelectingCalibrationMode = false;
  isSettingInversion = false;
  calibrationStep = 0;
  
  // Reinitialize compass
  if (qmcCompass.begin()) {
    activeCompass = &qmcCompass;
    logMessage("QMC5883L reinitialized after reset");
  } else if (hmcCompass.begin()) {
    activeCompass = &hmcCompass;
    logMessage("HMC5883L reinitialized after reset");
  } else {
    activeCompass = nullptr;
    logMessage("Compass reinitialization failed after reset");
  }
  
  // If we have a working compass, read initial heading
  if (activeCompass) {
    activeCompass->read();
    int initialHeading = activeCompass->getAzimuth();
    char dirStr[4];
    activeCompass->getDirection(dirStr, initialHeading);
    logMessage("Initial compass heading after reset: " + String(initialHeading) + "° (" + String(dirStr) + ")");
  }
}