#include "display_world_map.h"
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "display_manager.h"
#include <TinyGPS++.h>
#include "CompassInterface.h"
#include "main_globals.h"

extern Adafruit_SSD1306 display;
extern bool display_initialized;
extern const unsigned char world_map[];
extern float getLatitude();
extern float getLongitude();
extern bool privacyModeEnabled;
extern void drawPrivacyIndicator(Adafruit_SSD1306 &display);
extern bool compassInverted;
extern void logMessage(const String& msg);
extern DisplayMode currentDisplayMode;

void displayWorldMap() {
  if (!display_initialized) return;

  display.clearDisplay();
  display.drawBitmap(0, 0, world_map, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  if (gps.location.isValid()) {
    float lat = getLatitude();
    float lng = getLongitude();
    int posX = (int)((lng + 180.0) / 360.0 * SCREEN_WIDTH);
    if (lat > 85.0) lat = 85.0;
    if (lat < -85.0) lat = -85.0;
    float latRad = lat * PI / 180.0;
    float mercN = log(tan((PI/4) + (latRad/2)));
    int posY = (int)(SCREEN_HEIGHT/2 - (mercN * SCREEN_HEIGHT / (2*PI)));
    posX = constrain(posX, 0, SCREEN_WIDTH-1);
    posY = constrain(posY, 0, SCREEN_HEIGHT-1);
    if ((millis() / 500) % 2 == 0) {
      display.fillCircle(posX, posY, 3, SSD1306_WHITE);
      display.drawCircle(posX, posY, 4, SSD1306_WHITE);
    } else {
      display.drawCircle(posX, posY, 3, SSD1306_WHITE);
      display.drawCircle(posX, posY, 4, SSD1306_WHITE);
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
      display.drawLine(posX, posY, endX, endY, SSD1306_WHITE);
    }
  } else {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(2, SCREEN_HEIGHT - 8);
    display.print("No Fix");
  }
  if (privacyModeEnabled) {
    drawPrivacyIndicator(display);
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