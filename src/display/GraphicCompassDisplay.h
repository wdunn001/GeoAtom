#ifndef GRAPHIC_COMPASS_DISPLAY_H
#define GRAPHIC_COMPASS_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "CompassInterface.h"

class GraphicCompassDisplay {
public:
    GraphicCompassDisplay(Adafruit_SSD1306& displayRef, CompassInterface* compass);
    
    void showDisplay();
    void handleShortPress();
    void handleLongPress();
    
private:
    Adafruit_SSD1306& screen;
    CompassInterface* compass;
    bool showDegrees;
    
    void drawCompass(float heading);
    void drawHeadingLine(float heading);
    void drawCardinalDirections();
    void drawDegreeMarkers();
    void drawHeadingText(float heading);
};

#endif // GRAPHIC_COMPASS_DISPLAY_H 