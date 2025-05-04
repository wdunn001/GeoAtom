#include "display_world_map.h"
#include <Arduino.h>
#include "display_manager.h"
#include <TinyGPS++.h>
#include "CompassInterface.h"
#include "main_globals.h"

extern bool display_initialized;
extern const unsigned char world_map[];
extern float getLatitude();
extern float getLongitude();
extern bool privacyModeEnabled;
extern bool compassInverted;
extern void logMessage(const String& msg);
extern DisplayMode currentDisplayMode;

void displayWorldMap() {
  if (!display_initialized) return;

  int posX = 0, posY = 0;
  display.clearDisplay();
  display.drawXBMP(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, world_map);
  if (gps.location.isValid()) {
    float lat = getLatitude();
    float lng = getLongitude();
    posX = (int)((lng + 180.0) / 360.0 * SCREEN_WIDTH);
    if (lat > 85.0) lat = 85.0;
    if (lat < -85.0) lat = -85.0;
    float latRad = lat * PI / 180.0;
    float mercN = log(tan((PI/4) + (latRad/2)));
    posY = (int)(SCREEN_HEIGHT/2 - (mercN * SCREEN_HEIGHT / (2*PI)));
    posX = constrain(posX, 0, SCREEN_WIDTH-1);
    posY = constrain(posY, 0, SCREEN_HEIGHT-1);
    if ((millis() / 500) % 2 == 0) {
      display.drawDisc(posX, posY, 3);
      display.drawCircle(posX, posY, 4);
    } else {
      display.drawCircle(posX, posY, 3);
      display.drawCircle(posX, posY, 4);
    }
    if (activeCompass != nullptr) {
      activeCompass->read();
      int heading = activeCompass->getAzimuth();
      if (compassInverted) {
        heading = (heading + 180) % 360;
      }
      float radians = heading * PI / 180.0;
      int arrowLength = 8;
      int endX = posX + sin(radians) * arrowLength;
      int endY = posY - cos(radians) * arrowLength;
      display.drawLine(posX, posY, endX, endY);
    }
  } else {
    display.setFont(u8g2_font_ncenB08_tr);
    display.setDrawColor(1);
    display.setCursor(2, SCREEN_HEIGHT - 8);
    display.print("No Fix");
  }
  if (privacyModeEnabled) {
    display.drawDisc(posX, posY, 3);
  }
  display.display();
}

void handleShortPressWorldMap() {
  currentDisplayMode = DisplayMode::GPS_STATUS;
  logMessage("Display mode changed to: GPS_STATUS");
}

void handleLongPressWorldMap() {
  privacyModeEnabled = !privacyModeEnabled;
  logMessage("Privacy mode " + String(privacyModeEnabled ? "enabled" : "disabled"));
  displayWorldMap();
} 