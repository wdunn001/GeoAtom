#include "display_graphic_compass.h"
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "display_manager.h"
#include "CompassInterface.h"
#include <TinyGPS++.h>
#include "main_globals.h"

// Externs for globals used in these functions
extern Adafruit_SSD1306 display;
extern bool display_initialized;
extern bool compassInverted;
extern int altitudeCorrection;
extern bool privacyModeEnabled;
extern float getLatitude();
extern float getLongitude();
extern void drawPrivacyIndicator(Adafruit_SSD1306 &display);
extern DisplayMode currentDisplayMode;
extern void logMessage(const String& msg);

void displayGraphicCompass() {
  if (!display_initialized) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

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
    
    display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
    
    int endX = centerX + radius * cos(angleRad);
    int endY = centerY + radius * sin(angleRad);
    display.drawLine(centerX, centerY, endX, endY, SSD1306_WHITE);
    
    float arrowAngle1 = angleRad + radians(150);
    float arrowAngle2 = angleRad + radians(210);
    int arrowLength = 6;
    int arrow1X = endX + arrowLength * cos(arrowAngle1);
    int arrow1Y = endY + arrowLength * sin(arrowAngle1);
    int arrow2X = endX + arrowLength * cos(arrowAngle2);
    int arrow2Y = endY + arrowLength * sin(arrowAngle2);
    display.drawLine(endX, endY, arrow1X, arrow1Y, SSD1306_WHITE);
    display.drawLine(endX, endY, arrow2X, arrow2Y, SSD1306_WHITE);
    
    display.fillCircle(centerX, centerY, 2, SSD1306_WHITE);
    
    display.setCursor(centerX - 2, centerY - radius - 7);
    display.print("N");
    display.setCursor(centerX - 2, centerY + radius + 1);
    display.print("S");
    display.setCursor(centerX + radius + 2, centerY - 3);
    display.print("E");
    display.setCursor(centerX - radius - 7, centerY - 3);
    display.print("W");
  } else {
    display.setCursor(centerX - 8, centerY - 4);
    display.print("ERR");
  }

  if (activeCompass != nullptr) {
    display.setCursor(0, 0);
    display.print("Az ");
    display.print(heading);
  } else {
    display.setCursor(0, 0);
    display.print("Az ---");
  }
  
  if (gps.altitude.isValid()) {
    bool reliableAlt = gps.satellites.value() >= 5 && gps.hdop.isValid() && gps.hdop.hdop() < 2.5;
    float correctedAlt = gps.altitude.meters() + altitudeCorrection;
    
    if (reliableAlt) {
      String altStr = "Alt " + String(correctedAlt, 0) + "m";
      int16_t x1, y1;
      uint16_t w, h;
      display.getTextBounds(altStr, 0, 0, &x1, &y1, &w, &h);
      display.setCursor(SCREEN_WIDTH - w, 0);
      display.print(altStr);
      // Move speed display just under altitude
      if (gps.speed.isValid()) {
        bool reliableSpeed = gps.satellites.value() >= 4 && gps.hdop.isValid() && gps.hdop.hdop() < 3.0;
        float speedKmph = gps.speed.kmph();
        if (!reliableSpeed || speedKmph < 3.0) speedKmph = 0;
        if (speedKmph > 0) {
          String spdStr = String(speedKmph, 1) + "km/h";
          int16_t sx1, sy1; uint16_t sw, sh;
          display.getTextBounds(spdStr, 0, 0, &sx1, &sy1, &sw, &sh);
          display.setCursor(SCREEN_WIDTH - sw, h + 1); // Just under altitude
          display.print(spdStr);
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
      int16_t sx1, sy1; uint16_t sw, sh;
      display.getTextBounds(spdStr, 0, 0, &sx1, &sy1, &sw, &sh);
      display.setCursor(SCREEN_WIDTH - sw, 0); // Top right if no altitude
      display.print(spdStr);
    }
  }
  
  if (gps.location.isValid()) {
    // Use privacy-filtered coordinates via wrapper functions
    float lat = getLatitude(); 
    float lng = getLongitude();
    
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.println("Lat");
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print(String(lat, 5));
    
    String lngLabel = "Lng";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(lngLabel, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 16);
    display.println(lngLabel);
    
    String lngValue = String(lng, 5);
    display.getTextBounds(lngValue, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 8);
    display.print(lngValue);
  } else {
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.println("Lat");
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.println("No Fix");
    
    String lngLabel = "Lng";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(lngLabel, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 16);
    display.println(lngLabel);
    
    String noFix = "No Fix";
    display.getTextBounds(noFix, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 8);
    display.println(noFix);
  }
  
  // Draw privacy indicator if privacy mode is enabled
  if (privacyModeEnabled) {
    drawPrivacyIndicator(display);
  }

  display.display();
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