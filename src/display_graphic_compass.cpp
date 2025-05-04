#include "display_graphic_compass.h"
#include <Arduino.h>
#include "display_manager.h"
#include "CompassInterface.h"
#include <TinyGPS++.h>
#include "main_globals.h"
#include <U8g2lib.h>

// Externs for globals used in these functions
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;
extern bool display_initialized;
extern bool compassInverted;
extern int altitudeCorrection;
extern bool privacyModeEnabled;
extern float getLatitude();
extern float getLongitude();
extern DisplayMode currentDisplayMode;
extern void logMessage(const String& msg);
extern void drawPrivacyIndicator(U8G2 &display);

void displayGraphicCompass() {
  if (!display_initialized) return;

  int w = 0;
  display.clearBuffer();
  setDisplayDefaultStyle();

  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2;
  int radius = 22;
  int heading = 0;
  
  if (activeCompass != nullptr) {
    heading = activeCompass->getAzimuth();
    if (compassInverted) {
      heading = (heading + 180) % 360;
    }
    
    float angleRad = radians(270 - heading);
    
    display.drawCircle(centerX, centerY, radius);
    
    int endX = centerX + radius * cos(angleRad);
    int endY = centerY + radius * sin(angleRad);
    display.drawLine(centerX, centerY, endX, endY);
    
    float arrowAngle1 = angleRad + radians(150);
    float arrowAngle2 = angleRad + radians(210);
    int arrowLength = 6;
    int arrow1X = endX + arrowLength * cos(arrowAngle1);
    int arrow1Y = endY + arrowLength * sin(arrowAngle1);
    int arrow2X = endX + arrowLength * cos(arrowAngle2);
    int arrow2Y = endY + arrowLength * sin(arrowAngle2);
    display.drawLine(endX, endY, arrow1X, arrow1Y);
    display.drawLine(endX, endY, arrow2X, arrow2Y);
    
    display.drawDisc(centerX, centerY, 2);
    
    // Draw N S E W labels, centered
    int nWidth = display.getStrWidth("N");
    int sWidth = display.getStrWidth("S");
    int eWidth = display.getStrWidth("E");
    int wWidth = display.getStrWidth("W");
    int fontHeight = 8; // Assuming 8px font height
    // North (top center)
    display.drawStr(centerX - nWidth / 2, centerY - radius - 2, "N");
    // South (bottom center)
    display.drawStr(centerX - sWidth / 2, centerY + radius + fontHeight, "S");
    // East (right center)
    display.drawStr(centerX + radius + 4, centerY + fontHeight / 2, "E");
    // West (left center)
    display.drawStr(centerX - radius - wWidth - 4, centerY + fontHeight / 2, "W");
  } else {
    display.drawStr(centerX - 8, centerY - 4, "ERR");
  }

  if (activeCompass != nullptr) {
    display.drawStr(0, 10, (String("Az ") + String(heading)).c_str());
  } else {
    display.drawStr(0, 10, "Az ---");
  }
  
  if (gps.altitude.isValid()) {
    bool reliableAlt = gps.satellites.value() >= 5 && gps.hdop.isValid() && gps.hdop.hdop() < 2.5;
    float correctedAlt = gps.altitude.meters() + altitudeCorrection;
    if (reliableAlt) {
      String altStr = "Alt " + String(correctedAlt, 0) + "m";
      w = display.getStrWidth(altStr.c_str());
      display.drawStr(SCREEN_WIDTH - w, 10, altStr.c_str());
      // Move speed display just under altitude
      if (gps.speed.isValid()) {
        bool reliableSpeed = gps.satellites.value() >= 4 && gps.hdop.isValid() && gps.hdop.hdop() < 3.0;
        float speedKmph = gps.speed.kmph();
        if (!reliableSpeed || speedKmph < 3.0) speedKmph = 0;
        if (speedKmph > 0) {
          String spdStr = String(speedKmph, 1) + "km/h";
          w = display.getStrWidth(spdStr.c_str());
          display.drawStr(SCREEN_WIDTH - w, 20, spdStr.c_str());
        }
      }
    }
  }
  else if (gps.speed.isValid()) {
    // If no altitude, but speed is valid, show speed at top right
    bool reliableSpeed = gps.satellites.value() >= 4 && gps.hdop.isValid() && gps.hdop.hdop() < 3.0;
    float speedKmph = gps.speed.kmph();
    if (!reliableSpeed || speedKmph < 3.0) speedKmph = 0;
    if (speedKmph > 0) {
      String spdStr = String(speedKmph, 1) + "km/h";
      w = display.getStrWidth(spdStr.c_str());
      display.drawStr(SCREEN_WIDTH - w, 10, spdStr.c_str());
    }
  }
  
  // Draw GPS coordinates at the bottom
  if (gps.location.isValid()) {
    float lat = getLatitude();
    float lng = getLongitude();
    String latStr = String("Lat ") + String(lat, 5);
    String lngStr = String("Lng ") + String(lng, 5);
    int latY = SCREEN_HEIGHT - 2;
    int lngY = SCREEN_HEIGHT - 2;
    display.drawStr(0, latY, latStr.c_str());
    int lngWidth = display.getStrWidth(lngStr.c_str());
    display.drawStr(SCREEN_WIDTH - lngWidth, lngY, lngStr.c_str());
  } else {
    String latStr = "Lat No Fix";
    String lngStr = "Lng No Fix";
    int latY = SCREEN_HEIGHT - 2;
    int lngY = SCREEN_HEIGHT - 2;
    display.drawStr(0, latY, latStr.c_str());
    int lngWidth = display.getStrWidth(lngStr.c_str());
    display.drawStr(SCREEN_WIDTH - lngWidth, lngY, lngStr.c_str());
  }
  
  // Draw privacy indicator if privacy mode is enabled
  if (privacyModeEnabled) {
    drawPrivacyIndicator(display);
  }

  display.sendBuffer();
}

void handleShortPressGraphicCompass() {
    currentDisplayMode = DisplayMode::WORLD_MAP;
    logMessage("Display mode changed to: WORLD_MAP");
}

void handleLongPressGraphicCompass() {
  // Toggle compass inversion
  compassInverted = !compassInverted;
  logMessage("Compass display " + String(compassInverted ? "inverted" : "normal"));
} 