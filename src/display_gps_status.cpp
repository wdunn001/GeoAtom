#include "display_gps_status.h"
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <Preferences.h>
#include "display_manager.h"
#include "main_globals.h"

// Externs for globals used in these functions
extern Adafruit_SSD1306 display;
extern bool display_initialized;
extern TinyGPSPlus gps;
extern int gpsBaudRate;
extern int altitudeCorrection;
extern bool privacyModeEnabled;
extern float getLatitude();
extern float getLongitude();
extern DisplayMode currentDisplayMode;
extern void logMessage(const String& msg);
extern Preferences preferences;
extern const char* PREF_NAMESPACE;
extern const char* KEY_ALT_CORRECTION;
extern bool isSettingAltitudeCorrection;

void displayGPSStatusOnOLED() {
  if (!display_initialized) return;

  // Use static variables to hold the protocol string and its last update time
  static String displayedGpsProtocol = "Initializing...";
  static unsigned long lastProtocolUpdate = 0;
  const unsigned long PROTOCOL_UPDATE_INTERVAL = 5000; // 5 seconds

  // Add variables for packets and errors per second calculation
  static unsigned long lastPacketCount = 0;
  static unsigned long lastPacketTime = 0;
  static float packetsPerSecond = 0.0f;
  
  static unsigned long lastErrorCount = 0;
  static unsigned long lastErrorTime = 0;
  static float errorsPerSecond = 0.0f;

  display.clearDisplay(); 
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Center the title
  String title = "--- GPS Status ---";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 0);
  display.println(title);

  // Update GPS protocol string only every 5 seconds
  if (millis() - lastProtocolUpdate >= PROTOCOL_UPDATE_INTERVAL) {
    lastProtocolUpdate = millis();
    if (gps.charsProcessed() > 0) {
      String tempProtocol = "";
      if (gps.location.isUpdated()) tempProtocol += "GGA ";
      if (gps.date.isUpdated() || gps.time.isUpdated()) tempProtocol += "RMC ";
      if (gps.course.isUpdated()) tempProtocol += "VTG ";
      if (gps.satellites.isUpdated()) tempProtocol += "GSV "; // Added check for GSV
      if (gps.hdop.isValid()) tempProtocol += "GSA "; // Added check for GSA
      // GLL might not have a direct TinyGPS++ updated flag, assume active if others are
      if (tempProtocol.length() > 0) tempProtocol += "GLL "; 
      
      if (tempProtocol.length() == 0) tempProtocol = "NMEA";
      displayedGpsProtocol = tempProtocol + String(gpsBaudRate);
    } else {
      displayedGpsProtocol = "No GPS data";
    }
  }
  
  // Display the potentially cached protocol string
  display.setCursor(0, 10); // Adjusted Y position for better spacing
  display.println(displayedGpsProtocol);
  
  // Display Lat, Lng, and Sat (updated every display cycle)
  if (gps.location.isValid()) {
    // Use wrapper functions that handle privacy masking
    float lat = getLatitude();
    float lng = getLongitude();
    
    display.setCursor(0, 20); // Adjusted Y position
    display.print("Lat ");
    display.println(lat, 5);
    
    display.setCursor(0, 28); // Adjusted Y position
    display.print("Lng ");
    display.println(lng, 5);
  } else {
    display.setCursor(0, 20); // Adjusted Y position
    display.println("Lat No Fix");
    display.setCursor(0, 28); // Adjusted Y position
    display.println("Lng No Fix");
  }
  
  // Display satellite count
  display.setCursor(0, 36); // Adjusted Y position
  display.print("Sat ");
  display.println(gps.satellites.value());
  
  // Display altitude with correction
  if (gps.altitude.isValid()) {
    float correctedAlt = gps.altitude.meters() + altitudeCorrection;
    display.setCursor(64, 36); // Adjusted X position
    display.print("Alt ");
    display.print(correctedAlt, 0);
    display.println("m");
  }

  // Display NMEA stats 
  display.setCursor(0, 44); // Adjusted Y position

  // Calculate packets and errors per second
  unsigned long currentTime = millis();
  if (currentTime - lastPacketTime >= 1000) { // Update rate every second
    unsigned long currentPackets = gps.passedChecksum();
    unsigned long currentErrors = gps.failedChecksum();
    
    packetsPerSecond = (currentPackets - lastPacketCount) * 1000.0f / (currentTime - lastPacketTime);
    errorsPerSecond = (currentErrors - lastErrorCount) * 1000.0f / (currentTime - lastErrorTime);
    
    lastPacketCount = currentPackets;
    lastErrorCount = currentErrors;
    lastPacketTime = currentTime;
    lastErrorTime = currentTime;
  }

  display.print("Pkt/s ");
  display.print(packetsPerSecond, 1);
  display.print(" Err/s ");
  display.print(errorsPerSecond, 1);

  // Display course and speed in a single line
  if (gps.course.isValid() && gps.speed.isValid()) {
    display.setCursor(0, 52); // Adjusted Y position
    display.print("Hdg ");
    
    // Get direction abbreviation
    int course = gps.course.deg();
    String direction = "";
    if (course >= 337.5 || course < 22.5) direction = "N";
    else if (course >= 22.5 && course < 67.5) direction = "NE";
    else if (course >= 67.5 && course < 112.5) direction = "E";
    else if (course >= 112.5 && course < 157.5) direction = "SE";
    else if (course >= 157.5 && course < 202.5) direction = "S";
    else if (course >= 202.5 && course < 247.5) direction = "SW";
    else if (course >= 247.5 && course < 292.5) direction = "W";
    else direction = "NW";
    
    display.print(direction);
    display.print(" ");
    
    // Display speed
    float speedKmph = gps.speed.kmph();
    bool reliableSpeed = gps.satellites.value() >= 4 && gps.hdop.isValid() && gps.hdop.hdop() < 3.0;
    if (!reliableSpeed || speedKmph < 3.0) speedKmph = 0;
    display.print(speedKmph, 1);
    display.print("km/h");
  }

  // Draw privacy indicator if privacy mode is enabled
  if (privacyModeEnabled) {
    // Assume drawPrivacyIndicator is available
    extern void drawPrivacyIndicator(Adafruit_SSD1306 &display);
    drawPrivacyIndicator(display);
  }
  
  display.display();
}

void handleShortPressGPSStatus() {
  // Switch to next screen
  currentDisplayMode = DisplayMode::COMPASS_STATUS;
  logMessage("Display mode changed to: COMPASS_STATUS");
}

void handleLongPressGPSStatus() {
  // Enter altitude correction mode
  extern bool isSettingAltitudeCorrection;
  isSettingAltitudeCorrection = true;
  logMessage("Entering altitude correction mode...");
} 

// Altitude Correction Mode
void handleShortPressAltitudeCorrection() {
  // Short press in altitude correction mode: Increment correction
  altitudeCorrection += 10;
  logMessage("Altitude correction: " + String(altitudeCorrection) + "m");
}

void handleLongPressAltitudeCorrection() {
  // Long press in altitude correction mode: Save and exit
  preferences.begin(PREF_NAMESPACE, false);
  preferences.putInt(KEY_ALT_CORRECTION, altitudeCorrection);
  preferences.end();
  isSettingAltitudeCorrection = false;
  logMessage("Altitude correction saved: " + String(altitudeCorrection) + "m");
}