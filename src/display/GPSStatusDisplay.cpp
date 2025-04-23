#include "GPSStatusDisplay.h"

GPSStatusDisplay::GPSStatusDisplay(Adafruit_SSD1306& displayRef, TinyGPSPlus& gps)
    : screen(displayRef), gps(gps) {}

void GPSStatusDisplay::showDisplay() {
    screen.clearDisplay();
    screen.setTextSize(1);
    screen.setTextColor(SSD1306_WHITE);

    drawGPSData();
    drawSatelliteInfo();
    drawSignalStrength();

    screen.display();
}

void GPSStatusDisplay::handleShortPress() {
    // No special action for short press
}

void GPSStatusDisplay::handleLongPress() {
    // No special action for long press
}

void GPSStatusDisplay::drawGPSData() {
    if (gps.location.isValid()) {
        screen.setCursor(0, 0);
        screen.print("Lat: ");
        screen.println(gps.location.lat(), 6);
        screen.print("Lng: ");
        screen.println(gps.location.lng(), 6);
        
        if (gps.altitude.isValid()) {
            screen.print("Alt: ");
            screen.print(gps.altitude.meters(), 1);
            screen.println("m");
        }
    } else {
        screen.setCursor(0, 0);
        screen.println("No GPS Fix");
    }
}

void GPSStatusDisplay::drawSatelliteInfo() {
    screen.setCursor(0, 32);
    screen.print("Sats: ");
    screen.println(gps.satellites.value());
    
    if (gps.hdop.isValid()) {
        screen.print("HDOP: ");
        screen.println(gps.hdop.hdop());
    }
}

void GPSStatusDisplay::drawSignalStrength() {
    screen.setCursor(0, 48);
    screen.print("Fix Age: ");
    if (gps.location.age() < 3000) {
        screen.println(gps.location.age());
    } else {
        screen.println("OLD");
    }
    
    screen.print("Chars: ");
    screen.print(gps.charsProcessed());
    screen.print("/");
    screen.println(gps.sentencesWithFix());
} 