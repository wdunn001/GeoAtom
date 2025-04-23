#ifndef GPS_STATUS_DISPLAY_H
#define GPS_STATUS_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

class GPSStatusDisplay {
public:
    GPSStatusDisplay(Adafruit_SSD1306& displayRef, TinyGPSPlus& gps);
    void showDisplay();
    void handleShortPress();
    void handleLongPress();

private:
    Adafruit_SSD1306& screen;
    TinyGPSPlus& gps;
    void drawGPSData();
    void drawSatelliteInfo();
    void drawSignalStrength();
};

#endif // GPS_STATUS_DISPLAY_H 