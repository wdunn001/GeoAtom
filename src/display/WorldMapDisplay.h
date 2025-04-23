#ifndef WORLD_MAP_DISPLAY_H
#define WORLD_MAP_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

class WorldMapDisplay {
public:
    WorldMapDisplay(Adafruit_SSD1306& displayRef, TinyGPSPlus& gps);
    void showDisplay();
    void handleShortPress();
    void handleLongPress();

private:
    Adafruit_SSD1306& screen;
    TinyGPSPlus& gps;
    bool privacyModeEnabled;
    void drawWorldMap();
    void drawCurrentLocation();
    void drawScale();
    void drawCompassRose();
};

#endif // WORLD_MAP_DISPLAY_H 