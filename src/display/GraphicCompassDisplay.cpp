#include "GraphicCompassDisplay.h"

GraphicCompassDisplay::GraphicCompassDisplay(Adafruit_SSD1306& displayRef, CompassInterface* compass)
    : screen(displayRef), compass(compass), showDegrees(false) {}

void GraphicCompassDisplay::showDisplay() {
    screen.clearDisplay();
    
    if (compass != nullptr) {
        compass->read();
        float heading = compass->getAzimuth();
        
        drawCompass(heading);
        drawHeadingLine(heading);
        drawCardinalDirections();
        drawDegreeMarkers();
        drawHeadingText(heading);
    }
    
    screen.display();
}

void GraphicCompassDisplay::handleShortPress() {
    showDegrees = !showDegrees;
}

void GraphicCompassDisplay::handleLongPress() {
    // No special action for long press
}

void GraphicCompassDisplay::drawCompass(float heading) {
    const int centerX = screen.width() / 2;
    const int centerY = screen.height() / 2;
    const int radius = min(centerX, centerY) - 2;
    
    // Draw compass circle
    screen.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
    
    // Draw inner circle
    screen.drawCircle(centerX, centerY, radius - 2, SSD1306_WHITE);
}

void GraphicCompassDisplay::drawHeadingLine(float heading) {
    const int centerX = screen.width() / 2;
    const int centerY = screen.height() / 2;
    const int radius = min(centerX, centerY) - 2;
    
    float rad = (90 - heading) * PI / 180.0;
    int endX = centerX + radius * cos(rad);
    int endY = centerY - radius * sin(rad);
    
    // Draw heading line
    screen.drawLine(centerX, centerY, endX, endY, SSD1306_WHITE);
}

void GraphicCompassDisplay::drawCardinalDirections() {
    const int centerX = screen.width() / 2;
    const int centerY = screen.height() / 2;
    const int radius = min(centerX, centerY) - 2;
    
    // Draw N, E, S, W
    screen.setCursor(centerX - 2, centerY - radius + 2);
    screen.print("N");
    
    screen.setCursor(centerX + radius - 6, centerY - 2);
    screen.print("E");
    
    screen.setCursor(centerX - 2, centerY + radius - 10);
    screen.print("S");
    
    screen.setCursor(centerX - radius + 2, centerY - 2);
    screen.print("W");
}

void GraphicCompassDisplay::drawDegreeMarkers() {
    const int centerX = screen.width() / 2;
    const int centerY = screen.height() / 2;
    const int radius = min(centerX, centerY) - 2;
    
    for (int i = 0; i < 360; i += 30) {
        float rad = i * PI / 180.0;
        int x1 = centerX + (radius - 2) * cos(rad);
        int y1 = centerY - (radius - 2) * sin(rad);
        int x2 = centerX + radius * cos(rad);
        int y2 = centerY - radius * sin(rad);
        
        screen.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
    }
}

void GraphicCompassDisplay::drawHeadingText(float heading) {
    screen.setCursor(2, 2);
    screen.print("Heading: ");
    screen.print(heading, 1);
    screen.print(" deg");
} 