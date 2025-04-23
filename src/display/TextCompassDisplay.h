#ifndef TEXT_COMPASS_DISPLAY_H
#define TEXT_COMPASS_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "CompassInterface.h"

class TextCompassDisplay {
public:
    TextCompassDisplay(Adafruit_SSD1306& displayRef, CompassInterface* compass);
    void showDisplay();
    void handleShortPress();
    void handleLongPress();

private:
    Adafruit_SSD1306& screen;
    CompassInterface* compass;
    bool showDegrees;
};

#endif // TEXT_COMPASS_DISPLAY_H 