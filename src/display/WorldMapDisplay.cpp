#include "WorldMapDisplay.h"
#include "CompassInterface.h"

// World map bitmap data
static const unsigned char WORLD_MAP_BITMAP[] PROGMEM = {
    // ... existing world map bitmap data ...
};

WorldMapDisplay::WorldMapDisplay(Adafruit_SSD1306& displayRef, TinyGPSPlus& gps)
    : screen(displayRef), gps(gps), privacyModeEnabled(false) {
}

void WorldMapDisplay::showDisplay() {
    screen.clearDisplay();
    
    // Draw world map
    screen.drawBitmap(0, 0, WORLD_MAP_BITMAP, 128, 64, SSD1306_WHITE);
    
    if (gps.location.isValid() && !privacyModeEnabled) {
        // Convert GPS coordinates to screen coordinates
        int x = map(gps.location.lng(), -180, 180, 0, 128);
        int y = map(gps.location.lat(), 90, -90, 0, 64);
        
        // Draw current position
        screen.fillCircle(x, y, 2, SSD1306_WHITE);
    }
    
    // Draw privacy indicator if enabled
    if (privacyModeEnabled) {
        screen.setTextSize(1);
        screen.setTextColor(SSD1306_WHITE);
        screen.setCursor(0, 0);
        screen.print("P");
    }
    
    screen.display();
}

void WorldMapDisplay::handleShortPress() {
    // Toggle privacy mode
    privacyModeEnabled = !privacyModeEnabled;
}

void WorldMapDisplay::handleLongPress() {
    // No special action for long press in world map mode
}

void WorldMapDisplay::drawWorldMap() {
    screen.drawBitmap(0, 0, WORLD_MAP_BITMAP, 128, 64, SSD1306_WHITE);
}

void WorldMapDisplay::drawCurrentLocation() {
    if (gps.location.isValid() && !privacyModeEnabled) {
        int x = map(gps.location.lng(), -180, 180, 0, 128);
        int y = map(gps.location.lat(), 90, -90, 0, 64);
        screen.fillCircle(x, y, 2, SSD1306_WHITE);
    }
}

void WorldMapDisplay::drawScale() {
    // Not implemented yet
}

void WorldMapDisplay::drawCompassRose() {
    // Not implemented yet
} 