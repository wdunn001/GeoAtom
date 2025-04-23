#include "TextCompassDisplay.h"

TextCompassDisplay::TextCompassDisplay(Adafruit_SSD1306& displayRef, CompassInterface* compass)
    : screen(displayRef), compass(compass), showDegrees(false) {}

void TextCompassDisplay::showDisplay() {
    screen.clearDisplay();
    screen.setTextSize(1);
    screen.setTextColor(SSD1306_WHITE);

    if (compass != nullptr) {
        compass->read();
        float heading = compass->getAzimuth();
        
        // Draw heading in degrees
        screen.setCursor(0, 0);
        screen.print("Heading: ");
        screen.print(heading, 1);
        screen.println(" deg");
        
        // Draw cardinal direction
        char dirArray[4] = {' ', ' ', ' ', '\0'};
        compass->getDirection(dirArray, heading);
        screen.setCursor(0, 16);
        screen.print("Direction: ");
        screen.println(dirArray);
        
        // Draw raw sensor data
        screen.setCursor(0, 32);
        screen.print("X: ");
        screen.println(compass->getX());
        screen.print("Y: ");
        screen.println(compass->getY());
        screen.print("Z: ");
        screen.println(compass->getZ());
    } else {
        screen.setCursor(0, 0);
        screen.println("No compass");
        screen.println("detected");
    }

    screen.display();
}

void TextCompassDisplay::handleShortPress() {
    showDegrees = !showDegrees;
}

void TextCompassDisplay::handleLongPress() {
    // No special action for long press
} 