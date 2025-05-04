#include <U8g2lib.h>
#include <TinyGPS++.h>
#include <Preferences.h>
#include "display_manager.h"
#include "main_globals.h"

// Externs for globals used in these functions
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;
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

  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);

  // Center the title
  String title = "--- GPS Status ---";
  int16_t x = (128 - display.getStrWidth(title.c_str())) / 2;
  display.drawStr(x, 10, title.c_str());

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
  display.setCursor(0, 20); // Adjusted Y position for better spacing
  display.drawStr(0, 20, displayedGpsProtocol.c_str());
  
  // Display Lat, Lng, and Sat (updated every display cycle)
  if (gps.location.isValid()) {
    // Use wrapper functions that handle privacy masking
    float lat = getLatitude();
    float lng = getLongitude();
    
    display.setCursor(0, 28); // Adjusted Y position
    display.drawStr(0, 28, "Lat ");
    display.drawStr(24, 28, String(lat, 5).c_str());
    
    display.setCursor(0, 36); // Adjusted Y position
    display.drawStr(0, 36, "Lng ");
    display.drawStr(24, 36, String(lng, 5).c_str());
  } else {
    display.setCursor(0, 28); // Adjusted Y position
    display.drawStr(0, 28, "Lat No Fix");
    display.setCursor(0, 36); // Adjusted Y position
    display.drawStr(0, 36, "Lng No Fix");
  }
  
  // Display satellite count
  display.setCursor(0, 44); // Adjusted Y position
  display.drawStr(0, 44, "Sat ");
  display.drawStr(24, 44, String(gps.satellites.value()).c_str());
  
  // Display altitude with correction
  if (gps.altitude.isValid()) {
    float correctedAlt = gps.altitude.meters() + altitudeCorrection;
    display.setCursor(64, 44); // Adjusted X position
    display.drawStr(64, 44, "Alt ");
    display.drawStr(80, 44, String(correctedAlt, 0).c_str());
    display.drawStr(96, 44, "m");
  }

  // Display NMEA stats 
  display.setCursor(0, 52); // Adjusted Y position

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

  display.drawStr(0, 52, "Pkt/s ");
  display.drawStr(24, 52, String(packetsPerSecond, 1).c_str());
  display.drawStr(48, 52, " Err/s ");
  display.drawStr(64, 52, String(errorsPerSecond, 1).c_str());

  // Display course and speed in a single line
  if (gps.course.isValid() && gps.speed.isValid()) {
    display.setCursor(0, 60); // Adjusted Y position
    display.drawStr(0, 60, "Hdg ");
    
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
    
    display.drawStr(24, 60, direction.c_str());
    display.drawStr(48, 60, " ");
    
    // Display speed
    float speedKmph = gps.speed.kmph();
    bool reliableSpeed = gps.satellites.value() >= 4 && gps.hdop.isValid() && gps.hdop.hdop() < 3.0;
    if (!reliableSpeed || speedKmph < 3.0) speedKmph = 0;
    display.drawStr(64, 60, String(speedKmph, 1).c_str());
    display.drawStr(80, 60, "km/h");
  }

  // Draw privacy indicator if privacy mode is enabled
  if (privacyModeEnabled) {
    // Assume drawPrivacyIndicator is available
    extern void drawPrivacyIndicator(U8G2 &display);
    drawPrivacyIndicator(display);
  }
  
  display.sendBuffer();
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