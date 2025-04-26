#include <Arduino.h>
#include <HardwareSerial.h>
#include <M5Unified.h>
#include <TinyGPS++.h>
#include "CompassInterface.h"
#include <vector>
#include <String>
#include <Preferences.h>
#include "GPS_Configurator.h"
#include <map> // Add for std::map

// Include SSD1306 display libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Hand-drawn world map bitmap 128x64 pixels
static const unsigned char world_map[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x01, 0xe9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0e, 0x37, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0xc0, 0x00, 0x00, 0x00, 0x40, 0x0c, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xc0, 0x00, 0x00, 0x01, 0xc0, 0x34, 0x80, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x38, 0x00, 0x40, 0x40, 0x00, 0x00, 0x03, 0x80, 0x98, 0x80, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x1f, 0x00, 0x60, 0x80, 0x00, 0x00, 0x06, 0x01, 0x1b, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x03, 0x8f, 0x70, 0x21, 0x00, 0x00, 0x00, 0x0e, 0x06, 0x12, 0xf8, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x04, 0xf7, 0x78, 0x23, 0xe0, 0x00, 0x00, 0x0c, 0x3e, 0x00, 0x0c, 0xf0, 0x00, 0x00, 
	0x00, 0x10, 0x07, 0x35, 0x7e, 0x21, 0xa0, 0x00, 0x1c, 0x06, 0x7a, 0x00, 0x07, 0x88, 0x00, 0x00, 
	0x00, 0x47, 0x1d, 0xdb, 0x41, 0x14, 0x00, 0x00, 0x76, 0x13, 0xd0, 0x80, 0x00, 0x01, 0xf8, 0x00, 
	0x00, 0x80, 0x61, 0xbf, 0xf5, 0x80, 0x00, 0x00, 0x40, 0xbf, 0x7c, 0x08, 0x00, 0x00, 0x24, 0x00, 
	0x00, 0x80, 0x00, 0x62, 0x62, 0x80, 0x00, 0x00, 0x82, 0x70, 0x3c, 0x10, 0x00, 0x00, 0x01, 0x00, 
	0x01, 0xc0, 0x03, 0x05, 0xe2, 0xc0, 0x00, 0x01, 0x91, 0xc0, 0x30, 0x00, 0x00, 0x00, 0x07, 0x80, 
	0x00, 0xc0, 0x02, 0x03, 0xfc, 0xc0, 0x00, 0x01, 0x31, 0x80, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 
	0x00, 0xc0, 0x00, 0xc1, 0xaf, 0x80, 0x00, 0x02, 0x60, 0x00, 0x00, 0x00, 0x00, 0x01, 0x18, 0x00, 
	0x00, 0x8e, 0x00, 0x81, 0x0b, 0x80, 0x00, 0x04, 0x58, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x00, 
	0x00, 0x7c, 0xe0, 0x01, 0x08, 0x80, 0x00, 0x63, 0xc0, 0x02, 0x00, 0x00, 0x00, 0x7e, 0x80, 0x00, 
	0x00, 0x30, 0x20, 0x01, 0x88, 0x40, 0x00, 0x63, 0xf0, 0x00, 0x00, 0x01, 0x00, 0x82, 0x80, 0x00, 
	0x00, 0x60, 0x10, 0x00, 0x38, 0xe0, 0x00, 0xf3, 0xe0, 0x00, 0x00, 0x03, 0x01, 0x85, 0x00, 0x00, 
	0x00, 0x00, 0x08, 0x06, 0x18, 0x30, 0x00, 0xf6, 0x00, 0x00, 0x00, 0x06, 0x00, 0x42, 0x00, 0x00, 
	0x00, 0x00, 0x0c, 0x02, 0x18, 0xb0, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x00, 0x00, 
	0x00, 0x00, 0x06, 0x00, 0xe1, 0xb8, 0x00, 0x30, 0x00, 0x80, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x04, 0x00, 0x70, 0xc0, 0x00, 0x11, 0x87, 0x80, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x04, 0x00, 0x52, 0x00, 0x00, 0x01, 0xc6, 0x00, 0x00, 0x00, 0x06, 0x40, 0x00, 0x00, 
	0x00, 0x00, 0x04, 0x00, 0x06, 0x00, 0x00, 0x08, 0xfc, 0x80, 0x00, 0x00, 0x3c, 0x60, 0x00, 0x00, 
	0x00, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x2d, 0x5f, 0x20, 0x00, 0x00, 0x3c, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x03, 0x00, 0x08, 0x00, 0x00, 0x71, 0x81, 0x00, 0x00, 0x00, 0x2f, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xc0, 0x70, 0x00, 0x00, 0x80, 0x6f, 0x10, 0x00, 0x00, 0x17, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xc2, 0x10, 0x00, 0x00, 0x80, 0x01, 0x0a, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x72, 0x10, 0x00, 0x00, 0x00, 0x01, 0x8e, 0x40, 0x00, 0x30, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x12, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x82, 0x31, 0x43, 0x20, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0d, 0xc3, 0x00, 0x02, 0x00, 0x00, 0xcc, 0x02, 0x32, 0x30, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x01, 0x60, 0x00, 0x02, 0x00, 0x00, 0x78, 0x14, 0x18, 0x38, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x01, 0x00, 0x00, 0x38, 0x0e, 0x1e, 0x78, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x18, 0x20, 0x00, 0x8c, 0x00, 0x10, 0x06, 0x08, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x08, 0x04, 0x00, 0x01, 0x00, 0x20, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x10, 0x06, 0x00, 0x00, 0x00, 0x40, 0x00, 0x06, 0x18, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x03, 0xee, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x01, 0xfd, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x40, 0x00, 0x80, 0x18, 0x00, 0x00, 0xf8, 0xd8, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x40, 0x01, 0x80, 0xb8, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x20, 0x00, 0x00, 0x03, 0x58, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x80, 0x00, 0x81, 0x30, 0x00, 0x00, 0x04, 0x30, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x30, 0x04, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x46, 0x00, 0x00, 0x00, 0x20, 0x02, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x5c, 0x00, 0x00, 0x00, 0x20, 0x01, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x60, 0xc0, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x02, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x42, 0xe0, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x02, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x80, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x04, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x04, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00
};

// Forward declarations for custom functions
void updateDisplay(float lat, float lng, float alt, int heading);
void scanI2CDevices(void);
void displayLogOnOLED();
void displayCompassLogOnOLED();
void displayGraphicCompass();
void displayWorldMap();
void calculateAndApplyCalibration(); // New calibration function
void setDeclinationFromGPS(float lat, float lng); // Auto declination based on GPS position
void applyPrivacyFilter(); // Stub - Replaced by getLatitude() and getLongitude() wrappers

// Add this function to draw a more prominent privacy indicator
void drawPrivacyIndicator(Adafruit_SSD1306 &display) {
  extern bool privacyModeEnabled; // Fix linter error by adding external declaration
  
  if (privacyModeEnabled) {
    // Draw a padlock icon at top-right corner
    int x = display.width() - 10;
    int y = 2;
    
    // Lock body
    display.fillRect(x, y + 3, 8, 6, SSD1306_WHITE);
    
    // Lock shackle
    display.drawRect(x + 1, y, 6, 4, SSD1306_WHITE);
    
    // Optional: "P" for privacy
    display.drawPixel(x + 2, y + 5, SSD1306_BLACK);
    display.drawPixel(x + 3, y + 4, SSD1306_BLACK);
    display.drawPixel(x + 4, y + 5, SSD1306_BLACK);
    display.drawPixel(x + 3, y + 6, SSD1306_BLACK);
    display.drawPixel(x + 2, y + 7, SSD1306_BLACK);
  }
}

// GPS setup
TinyGPSPlus gps;
HardwareSerial GPS(1);  // Using UART1 for GPS
HardwareSerial Host(2); // Using UART2 for NMEA forwarding to PC/TTL converter
unsigned long gpsBaudRate = 0; // Variable to store GPS baud rate
int altitudeCorrection = 0; // Altitude correction factor in meters - Set to 0
CompassInterface* activeCompass = nullptr;
QMC5883LCompassImpl qmcCompass;
HMC5883LCompassImpl hmcCompass;
bool isCalibrating = false;
bool isSettingDeclination = false; // New flag for declination setting mode
bool isSelectingCalibrationMode = false; // Flag for calibration mode selection
bool isSettingInversion = false; // Flag for compass inversion setting
bool compassInverted = false; // Whether compass display should be inverted
int calibrationStep = 0;
int calibrationPoints = 8; // Default to 8-point calibration
int calX[16]; // Stores calibration points (up to 16)
int calY[16]; // Stores calibration points (up to 16)
float currentDeclination = 0.0f; // Current declination value in radians
int calibrationModeIndex = 0; // Index for current selection in calibration mode
bool isSettingAltitudeCorrection = false; // Flag for altitude correction mode
bool privacyModeEnabled = false; // Flag to toggle privacy mode for coordinates

// Calibration mode options
const int NUM_CALIBRATION_MODES = 4; // 4-point, 8-point, 16-point, Cancel
const char* CALIBRATION_MODE_NAMES[NUM_CALIBRATION_MODES] = {
  "4-Point Cal", "8-Point Cal", "16-Point Cal", "Cancel"
};
const int CALIBRATION_MODE_POINTS[NUM_CALIBRATION_MODES] = {
  4, 8, 16, 0 // 0 points means cancel
};

// SSD1306 display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Reset pin not used on most modules
#define OLED_ADDR 0x3C  // Common I2C address for SSD1306

// Create display instance without initializing it yet
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool display_initialized = false;  // Flag to track if display is initialized

// Log buffer setup
const size_t MAX_LOG_LINES = 15; // Buffer size (can adjust if needed)
std::vector<String> log_buffer; // Historical log buffer
// bool show_log_on_oled = false; // Replaced by currentDisplayMode

// Display Modes Enum
enum class DisplayMode { GPS_STATUS, COMPASS_STATUS, GRAPHIC_COMPASS, WORLD_MAP };
DisplayMode currentDisplayMode = DisplayMode::GRAPHIC_COMPASS; // Start in graphic compass mode

// Variables to hold latest status for fixed display
String latest_gps_status = "GPS: Initializing...";
String latest_compass_raw = "Compass: Initializing...";

// Preferences object and keys for storing calibration
Preferences preferences;
const char* PREF_NAMESPACE = "compassCal";
const char* KEY_CAL_VALID = "calValid";
const char* KEY_X_OFFSET = "xOffset";
const char* KEY_Y_OFFSET = "yOffset";
const char* KEY_Z_OFFSET = "zOffset";
const char* KEY_X_SCALE = "xScale";
const char* KEY_Y_SCALE = "yScale";
const char* KEY_Z_SCALE = "zScale";
const char* KEY_DECLINATION = "declRad"; // New key for declination
const char* KEY_COMPASS_INVERTED = "compInv"; // New key for compass inversion
const char* KEY_ALT_CORRECTION = "altCorr"; // Key for altitude correction
const char* KEY_HMC_CAL_VALID = "hmcCalValid"; // Key for HMC calibration validity
const char* KEY_HMC_X_OFFSET = "hmcXOffset"; // Key for HMC X offset
const char* KEY_HMC_Y_OFFSET = "hmcYOffset"; // Key for HMC Y offset
const char* KEY_HMC_Z_OFFSET = "hmcZOffset"; // Key for HMC Z offset
const char* KEY_HMC_X_SCALE = "hmcXScale"; // Key for HMC X scale
const char* KEY_HMC_Y_SCALE = "hmcYScale"; // Key for HMC Y scale
const char* KEY_HMC_Z_SCALE = "hmcZScale"; // Key for HMC Z scale

// Add these configuration flags near the other configuration flags
bool useM5StackSmoothing = true;      // Whether to use M5Stack's built-in smoothing (DEFAULT ON)
bool useM5StackInterference = true;    // Whether to use M5Stack's built-in magnetic interference detection (DEFAULT ON)

// Add these configuration keys with the other preference keys
const char* KEY_USE_M5_SMOOTHING = "useM5Smooth";     // Key for M5Stack smoothing preference
const char* KEY_USE_M5_INTERFERENCE = "useM5Interf";  // Key for M5Stack interference preference

// Pin definitions based on physical connections
// --- Verified Working Configuration for BN-880 & M5Atom Echo ---
// I2C pins - shared between compass (BN-880) and display (SSD1306)
#define I2C_SDA 25 // Grove Pin 1
#define I2C_SCL 21 // Grove Pin 2

// GPS module connections (BN-880 UART)
#define GPS_RX 23 // ESP32 RX pin connected to BN-880 TX
#define GPS_TX 33  // ESP32 TX pin connected to BN-880 RX

// TTL to RS232 level converter connections (Optional Host Output)
#define HOST_RX 22  // ESP32 RX pin connected to TTL TX
#define HOST_TX 19  // ESP32 TX pin connected to TTL RX
// --------------------------------------------------------------

// Logging function - sends to Host serial and adds to buffer
void logMessage(const String& msg) {
  Host.println(msg); // Keep sending to serial
  // Add message to buffer, managing size
  if (log_buffer.size() >= MAX_LOG_LINES) {
    log_buffer.erase(log_buffer.begin()); // Remove the oldest message
  }
  log_buffer.push_back(msg); // Add the new message
}

// Set the declination based on compass direction relative to true north from GPS
void setDeclinationFromGPS(float lat, float lng) {
  // Only attempt this if we have a valid GPS fix and compass
  if (!gps.location.isValid() || activeCompass == nullptr) {
    logMessage("Cannot set declination: Need valid GPS and compass");
    return;
  }
  
  // Get current magnetic heading
  activeCompass->read();
  int magneticHeading = activeCompass->getAzimuth();
  
  // Calculate true heading from GPS course (requires movement)
  float trueHeading = gps.course.deg();
  
  // We need a valid course, reliable signal quality, and some movement to get accurate true heading
  bool reliableGPS = gps.satellites.value() >= 5 && gps.hdop.isValid() && gps.hdop.hdop() < 2.0;
  
  if (!gps.course.isValid() || !reliableGPS || gps.speed.knots() < 2.0) {
    logMessage("Cannot set declination: Need reliable movement for true heading");
    return;
  }
  
  // Calculate declination: difference between true and magnetic headings
  float declination = trueHeading - magneticHeading;
  
  // Normalize to -180 to 180 range
  while (declination > 180) declination -= 360;
  while (declination < -180) declination += 360;
  
  // Convert to radians
  currentDeclination = radians(declination);
  
  // Set and save declination
  if (activeCompass == &hmcCompass) {
    hmcCompass.setDeclination(currentDeclination);
    
    // Save to NVM
    preferences.begin(PREF_NAMESPACE, false);
    preferences.putFloat(KEY_DECLINATION, currentDeclination);
    preferences.end();
    
    logMessage("Auto-set declination to: " + String(declination, 1) + " degrees");
  } else {
    logMessage("Note: Auto-declination only applies to HMC5883L compass");
  }
}

// Add these declarations with other global variables
float smoothedHeading = 0.0f;  // Smoothed compass heading
float ALPHA = 0.0f;     // Smoothing factor (0.0-1.0): lower = more smoothing, higher = more responsive
// Compass raw value smoothing
int smoothedCompassX = 0;      // Smoothed compass X value
int smoothedCompassY = 0;      // Smoothed compass Y value 
int smoothedCompassZ = 0;      // Smoothed compass Z value
float COMPASS_XYZ_ALPHA = 0.0f; // Smoothing factor for raw compass values
bool compassValuesInitialized = false; // Flag to initialize compass values
bool compassStabilized = false; // Flag to indicate compass has stabilized after startup
unsigned long compassStartupTime = 0; // Timestamp when compass initialization started
const unsigned long COMPASS_STABILIZATION_TIME = 3000; // Wait 3 seconds for compass to stabilize

// Wrapper functions for GPS coordinates that apply privacy mask when needed
float getLatitude() {
  if (!gps.location.isValid()) {
    return 0.0f;
  }
  
  float rawLat = gps.location.lat();
  
  if (privacyModeEnabled) {
    // Apply privacy mask - keep only first digit
    float absLat = abs(rawLat);
    int latInt = static_cast<int>(absLat);
    String latStr = String(latInt);
    
    if (latInt == 0) {
      // If latitude is less than 1, set to 0
      return (rawLat < 0) ? -0.0f : 0.0f;
    } else {
      // Extract first digit and set rest to zeros
      int firstDigit = (latStr.length() > 0) ? (latStr[0] - '0') : 0;
      int zerosToAdd = latStr.length() - 1;
      
      // Reconstruct latitude with first digit only
      float maskedLat = firstDigit;
      for (int i = 0; i < zerosToAdd; i++) {
        maskedLat *= 10;
      }
      
      // Preserve sign
      return (rawLat < 0) ? -maskedLat : maskedLat;
    }
  }
  
  // No privacy mode - return raw value
  return rawLat;
}

float getLongitude() {
  if (!gps.location.isValid()) {
    return 0.0f;
  }
  
  float rawLng = gps.location.lng();
  
  if (privacyModeEnabled) {
    // Apply privacy mask - keep only first digit
    float absLng = abs(rawLng);
    int lngInt = static_cast<int>(absLng);
    String lngStr = String(lngInt);
    
    if (lngInt == 0) {
      // If longitude is less than 1, set to 0
      return (rawLng < 0) ? -0.0f : 0.0f;
    } else {
      // Extract first digit and set rest to zeros
      int firstDigit = (lngStr.length() > 0) ? (lngStr[0] - '0') : 0;
      int zerosToAdd = lngStr.length() - 1;
      
      // Reconstruct longitude with first digit only
      float maskedLng = firstDigit;
      for (int i = 0; i < zerosToAdd; i++) {
        maskedLng *= 10;
      }
      
      // Preserve sign
      return (rawLng < 0) ? -maskedLng : maskedLng;
    }
  }
  
  // No privacy mode - return raw value
  return rawLng;
}

// Function to apply privacy filter to GPS coordinates
void applyPrivacyFilter() {
  // Function no longer needed - replaced by getLatitude() and getLongitude() wrappers
}

// UI State Management - Centralized Button Handling
// Define all possible UI states
enum class UIState {
  // Main display modes
  GPS_STATUS_SCREEN,
  COMPASS_STATUS_SCREEN,
  GRAPHIC_COMPASS_SCREEN,
  WORLD_MAP_SCREEN,
  
  // Configuration modes
  ALTITUDE_CORRECTION_MODE,
  CALIBRATION_MODE_SELECTION,
  CALIBRATING_COMPASS,
  SETTING_DECLINATION,
  SETTING_INVERSION
};

// Current UI state
UIState currentUIState = UIState::GRAPHIC_COMPASS_SCREEN;

// Function to get the current UI state based on display mode and flags
UIState determineCurrentUIState() {
  // First determine if we're in a special mode
  // Use a single variable to hold the current mode state
  UIState detectedState;
  
  // Use a switch to determine the state based on configuration flags
  // Configuration modes take precedence over display modes
  if (isSettingAltitudeCorrection) {
    detectedState = UIState::ALTITUDE_CORRECTION_MODE;
  }
  else if (isSelectingCalibrationMode) {
    detectedState = UIState::CALIBRATION_MODE_SELECTION;
  }
  else if (isCalibrating) {
    detectedState = UIState::CALIBRATING_COMPASS;
  }
  else if (isSettingDeclination) {
    detectedState = UIState::SETTING_DECLINATION;
  }
  else if (isSettingInversion) {
    detectedState = UIState::SETTING_INVERSION;
  }
  else {
    // Not in configuration mode, so use display mode
    switch (currentDisplayMode) {
      case DisplayMode::GPS_STATUS:
        detectedState = UIState::GPS_STATUS_SCREEN;
        break;
      case DisplayMode::COMPASS_STATUS:
        detectedState = UIState::COMPASS_STATUS_SCREEN;
        break;
      case DisplayMode::GRAPHIC_COMPASS:
        detectedState = UIState::GRAPHIC_COMPASS_SCREEN;
        break;
      case DisplayMode::WORLD_MAP:
        detectedState = UIState::WORLD_MAP_SCREEN;
        break;
      default:
        detectedState = UIState::GRAPHIC_COMPASS_SCREEN;
        break;
    }
  }
  
  return detectedState;
}

// Function prototypes for button handlers
void handleShortPressGPSStatus();
void handleLongPressGPSStatus();
void handleShortPressCompassStatus();
void handleLongPressCompassStatus();
void handleShortPressGraphicCompass();
void handleLongPressGraphicCompass();
void handleShortPressWorldMap();
void handleLongPressWorldMap();
void handleShortPressAltitudeCorrection();
void handleLongPressAltitudeCorrection();
void handleShortPressCalibrationModeSelection();
void handleLongPressCalibrationModeSelection();
void handleShortPressCalibrating();
void handleLongPressCalibrating();
void handleShortPressDeclination();
void handleLongPressDeclination();
void handleShortPressInversion();
void handleLongPressInversion();

// Before the loop() function, add a new debugging variable to diagnose compass issues
bool enableCompassDebug = true;   // Set to true to output detailed compass diagnostics
unsigned long lastCompassDebugOutput = 0; // Timestamp for last compass debug output

// Compass interference mitigation
// Configuration flags - set custom ones to false by default
bool enableOutlierRejection = false;      // Reject sudden large jumps in heading (DEFAULT OFF)
bool enableNoiseThreshold = false;        // Ignore small changes below threshold (DEFAULT OFF)
bool enableGpsFusion = false;             // Blend GPS course when moving (DEFAULT OFF)
bool enableMagneticInterference = false;  // Detect and compensate for interference (DEFAULT OFF)

// Constants for interference mitigation
int MAX_HEADING_JUMP = 0;           // Maximum allowed heading change in degrees per reading
int HEADING_NOISE_THRESHOLD = 0;    // Degrees - changes smaller than this are ignored
float MIN_GPS_CONFIDENCE = 0.0;     // Minimum confidence to start using GPS course
float MAGNETIC_VARIANCE_THRESHOLD = 0.0; // Threshold for magnetic field variance indicating interference

// Runtime variables for interference detection
bool lastHeadingValid = false;             // Flag for first reading
bool highInterferenceDetected = false;     // Current interference status
bool usingGpsCourse = false;               // Whether GPS course is currently being used
bool fastTurningDetected = false;          // Flag for fast turning detection
float turningRate = 0.0f;                  // Detected turning rate in degrees per second

// Keys for storing interference mitigation settings
const char* KEY_OUTLIER_REJECTION = "outRej";
const char* KEY_NOISE_THRESHOLD = "noiseThresh";
const char* KEY_GPS_FUSION = "gpsFusion";
const char* KEY_MAG_INTERFERENCE = "magInt";

// GPS smoothing variables
float smoothedGpsSpeed = 0.0f;   // Smoothed GPS speed in km/h
float smoothedGpsCourse = 0.0f;  // Smoothed GPS course in degrees
const float GPS_SPEED_ALPHA = 0.2f;   // GPS speed smoothing factor
const float GPS_COURSE_ALPHA_MIN = 0.05f; // Course smoothing at low speeds
const float GPS_COURSE_ALPHA_MAX = 0.3f;  // Course smoothing at high speeds
const float GPS_SPEED_MIN = 3.0f;  // Minimum speed in km/h to consider course reliable
const float GPS_SPEED_MAX = 20.0f; // Speed at which to use max course alpha

// Update timing constants for optimal performance
const unsigned long COMPASS_SAMPLE_INTERVAL = 10;   // Sample compass at 100Hz (10ms)
const unsigned long DISPLAY_UPDATE_INTERVAL = 33;   // Update display at 30Hz (33ms)

// Function prototypes
void handleShortPressGPSStatus();
void handleLongPressGPSStatus();
void handleShortPressCompassStatus();
void handleLongPressCompassStatus();
void handleShortPressGraphicCompass();
void handleLongPressGraphicCompass();
void scanI2CDevices();

// Define a type for button handler functions for clarity
typedef void (*ButtonHandlerFunc)();

// Define a struct to hold both short and long press handlers
struct ButtonHandlers {
    ButtonHandlerFunc shortPressHandler;
    ButtonHandlerFunc longPressHandler;
};

// Create a map from UIState to button handlers
std::map<UIState, ButtonHandlers> buttonHandlerMap;

// Add these declarations near the top with other variables
const unsigned long GPS_INIT_TIMEOUT = 5000;  // 5 second timeout for GPS init
const unsigned long GPS_CONFIG_RETRY_DELAY = 100; // 100ms between retries
const uint8_t MAX_GPS_INIT_RETRIES = 3;  // Maximum number of initialization attempts
bool gpsInitialized = false;  // Track if GPS was successfully initialized

// Add these variables near the top with other global variables
static unsigned long lastPacketCount = 0;
static unsigned long lastPacketTime = 0;
static float packetsPerSecond = 0.0f;

void setup() {
  // Initialize M5 hardware with minimal configuration
  auto cfg = M5.config();
  cfg.serial_baudrate = 0;  // Disable M5 serial
  M5.begin(cfg);

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialize display first
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // If display fails, we can't show anything
    return;
  }
  display_initialized = true;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Starting...");
  display.display();


  // Initialize Host serial for debug
  Host.begin(4800, SERIAL_8N1, HOST_RX, HOST_TX);
  logMessage("System initializing...");

  // --- Initialize GPS with Configuration --- 
  logMessage("Initializing GPS module...");
   // Initialize GPS with validation and retry logic
  logMessage("Initializing GPS module...");
  gpsBaudRate = 115200;  // Start with default baud rate
  
  bool gpsInitSuccess = false; // **** THIS IS THE ONLY DECLARATION ****
  uint8_t initAttempts = 0;
 
  while (!gpsInitSuccess && initAttempts < MAX_GPS_INIT_RETRIES) {
    initAttempts++;
    logMessage("GPS init attempt " + String(initAttempts) + " of " + String(MAX_GPS_INIT_RETRIES));
    
    // Initialize GPS serial
    GPS.begin(gpsBaudRate, SERIAL_8N1, GPS_RX, GPS_TX);
    
    // Wait for GPS to be ready
    unsigned long startTime = millis();
    bool receivedData = false;
    
    while (millis() - startTime < GPS_INIT_TIMEOUT) {
      if (GPS.available()) {
        receivedData = true;
        break;
      }
      delay(10);
    }
    
    if (!receivedData) {
      logMessage("No response from GPS, retrying...");
      GPS.end();
      delay(GPS_CONFIG_RETRY_DELAY);
      continue;
    }
    
    // Configure GPS
    GPSConfigurator gpsConfig(GPS);
    bool configSuccess = true;  // Track if all configuration steps succeed
    
    
    // Set dynamic model to Portable (0)
    if (!gpsConfig.setDynamicModel(0)) {
      logMessage("Failed to set GPS dynamic model");
      configSuccess = false;
    }
    
    // Enable essential NMEA messages
    if (!gpsConfig.enableNmeaMessage(0xF0, 0x00, true) ||  // GGA - Fix data
        !gpsConfig.enableNmeaMessage(0xF0, 0x04, true) ||  // RMC - Recommended minimum data
        !gpsConfig.enableNmeaMessage(0xF0, 0x05, true) ||  // VTG - Vector track and speed
        !gpsConfig.enableNmeaMessage(0xF0, 0x02, true) ||  // GSA - DOP and active satellites
        !gpsConfig.enableNmeaMessage(0xF0, 0x03, true) ||  // GSV - Satellites in view
        !gpsConfig.enableNmeaMessage(0xF0, 0x01, true)) {  // GLL - Geographic position
      logMessage("Failed to configure NMEA messages");
      configSuccess = false;
    }
    
    // Save configuration if all steps succeeded
    if (configSuccess) {
      if (gpsConfig.saveConfiguration()) {
        logMessage("GPS configuration saved successfully");
        gpsInitSuccess = true;
        break;
      } else {
        logMessage("Failed to save GPS configuration");
      }
    }
    
    // If we get here, configuration failed
    GPS.end();
    delay(GPS_CONFIG_RETRY_DELAY);
  }
  
  if (gpsInitSuccess) {
    gpsInitialized = true;
    logMessage("GPS initialized successfully");
    logMessage("GPS Configuration:");
    logMessage("- Baud Rate: " + String(gpsBaudRate));
    logMessage("- Update Rate: 10Hz");
    logMessage("- Dynamic Model: Portable");
    logMessage("- NMEA Messages: GGA, RMC, VTG, GSA, GSV, GLL");
    logMessage("  * GGA: Fix data (time, position, fix type)");
    logMessage("  * RMC: Recommended minimum (pos, vel, time)");
    logMessage("  * VTG: Vector track and ground speed");
    logMessage("  * GSA: DOP and active satellites");
    logMessage("  * GSV: Satellites in view");
    logMessage("  * GLL: Geographic position");
  } else {
    logMessage("*** WARNING: GPS initialization failed after " + String(MAX_GPS_INIT_RETRIES) + " attempts ***");
    logMessage("System will continue but GPS functionality may be limited");
  }


  // Try to initialize compass
  if (qmcCompass.begin()) {
      activeCompass = &qmcCompass;
      logMessage("QMC5883L Initialized");
  } else if (hmcCompass.begin()) {
      activeCompass = &hmcCompass;
      logMessage("HMC5883L Initialized");
  } else {
      logMessage("Compass Init Failed");
  }

  // Load preferences OR apply defaults
  preferences.begin(PREF_NAMESPACE, true); // Open read-only first to check if keys exist

  // Load M5Stack settings, default to true if not found
  useM5StackSmoothing = preferences.getBool(KEY_USE_M5_SMOOTHING, true);
  useM5StackInterference = preferences.getBool(KEY_USE_M5_INTERFERENCE, true);
  
  // Keep custom features disabled (defaults to false)
  enableOutlierRejection = preferences.getBool(KEY_OUTLIER_REJECTION, false);
  enableNoiseThreshold = preferences.getBool(KEY_NOISE_THRESHOLD, false);
  enableGpsFusion = preferences.getBool(KEY_GPS_FUSION, false);
  enableMagneticInterference = preferences.getBool(KEY_MAG_INTERFERENCE, false);

  // Load related thresholds/parameters (these can still be loaded even if algo is off)
  MAX_HEADING_JUMP = preferences.getInt("maxHeadingJump", 0);
  HEADING_NOISE_THRESHOLD = preferences.getInt("headingNoiseThresh", 0);
  MAGNETIC_VARIANCE_THRESHOLD = preferences.getFloat("magVarThresh", 0.0f);
  ALPHA = preferences.getFloat("compassAlpha", 0.0f);
  COMPASS_XYZ_ALPHA = preferences.getFloat("compassXYZAlpha", 0.0f);
  
  // Load other settings (declination, inversion, etc.)
  compassInverted = preferences.getBool(KEY_COMPASS_INVERTED, false);
  altitudeCorrection = preferences.getInt(KEY_ALT_CORRECTION, 0);
  if (activeCompass == &hmcCompass) {
      currentDeclination = preferences.getFloat(KEY_DECLINATION, 0.0f);
      hmcCompass.setDeclination(currentDeclination);
  }
  // Note: We don't load calibration here in the simplified setup

  preferences.end();

  // Set initial display mode
  currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
  
  // Log the active settings
  logMessage("--- Active Settings ---");
  logMessage("M5 Smoothing: " + String(useM5StackSmoothing ? "ON" : "OFF"));
  logMessage("M5 Interference: " + String(useM5StackInterference ? "ON" : "OFF"));
  logMessage("Custom Smoothing: OFF");
  logMessage("Custom Interference: OFF");
  logMessage("-----------------------");

  // Show ready message
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("System Ready");
  display.display();
  delay(1000);  // Show ready message for 1 second

  // Initialize the button handler map
  buttonHandlerMap[UIState::GPS_STATUS_SCREEN] = ButtonHandlers{handleShortPressGPSStatus, handleLongPressGPSStatus};
  buttonHandlerMap[UIState::COMPASS_STATUS_SCREEN] = ButtonHandlers{handleShortPressCompassStatus, handleLongPressCompassStatus};
  buttonHandlerMap[UIState::GRAPHIC_COMPASS_SCREEN] = ButtonHandlers{handleShortPressGraphicCompass, handleLongPressGraphicCompass};
  buttonHandlerMap[UIState::WORLD_MAP_SCREEN] = ButtonHandlers{handleShortPressWorldMap, handleLongPressWorldMap};
  buttonHandlerMap[UIState::ALTITUDE_CORRECTION_MODE] = ButtonHandlers{handleShortPressAltitudeCorrection, handleLongPressAltitudeCorrection};
  buttonHandlerMap[UIState::CALIBRATION_MODE_SELECTION] = ButtonHandlers{handleShortPressCalibrationModeSelection, handleLongPressCalibrationModeSelection};
  buttonHandlerMap[UIState::CALIBRATING_COMPASS] = ButtonHandlers{handleShortPressCalibrating, handleLongPressCalibrating};
  buttonHandlerMap[UIState::SETTING_DECLINATION] = ButtonHandlers{handleShortPressDeclination, handleLongPressDeclination};
  buttonHandlerMap[UIState::SETTING_INVERSION] = ButtonHandlers{handleShortPressInversion, handleLongPressInversion};
  
  // Initialize preferences
  preferences.begin(PREF_NAMESPACE, true); // Open read-only
  
  // Load interference mitigation settings - initialize all to false
  enableOutlierRejection = false;
  enableNoiseThreshold = false;
  enableGpsFusion = false;
  enableMagneticInterference = false;
  
  // Load interference thresholds but keep algorithms disabled
  MAX_HEADING_JUMP = preferences.getInt("maxHeadingJump", 0);
  HEADING_NOISE_THRESHOLD = preferences.getInt("headingNoiseThresh", 0);
  MAGNETIC_VARIANCE_THRESHOLD = preferences.getFloat("magVarThresh", 0.0f);
  
  // Load smoothing parameters but keep algorithms disabled
  ALPHA = preferences.getFloat("compassAlpha", 0.0f);
  COMPASS_XYZ_ALPHA = preferences.getFloat("compassXYZAlpha", 0.0f);
  
  preferences.end();
  
  // Initialize display
  logMessage("Initializing display...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    logMessage("SSD1306 allocation failed");
  } else {
    display_initialized = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("GPS Navigation");
    display.println("-------------");
    display.println("Initializing...");
    display.display();
  }
  
  // Try to initialize QMC5883L first
  if (qmcCompass.begin()) {
    logMessage("QMC5883L initialized successfully.");
    activeCompass = &qmcCompass;
    
    // Load calibration from preferences
    preferences.begin(PREF_NAMESPACE, true); // Open read-only
    bool calValid = preferences.getBool(KEY_CAL_VALID, false);
    
    if (calValid) {
      float x_offset = preferences.getFloat(KEY_X_OFFSET, 0.0f);
      float y_offset = preferences.getFloat(KEY_Y_OFFSET, 0.0f);
      float z_offset = preferences.getFloat(KEY_Z_OFFSET, 0.0f);
      float x_scale = preferences.getFloat(KEY_X_SCALE, 1.0f);
      float y_scale = preferences.getFloat(KEY_Y_SCALE, 1.0f);
      float z_scale = preferences.getFloat(KEY_Z_SCALE, 1.0f);
      preferences.end();

      qmcCompass.setCalibrationOffsets(x_offset, y_offset, z_offset);
      qmcCompass.setCalibrationScales(x_scale, y_scale, z_scale);
      logMessage("Loaded QMC5883L compass calibration from NVM.");
    } else {
      preferences.end();
      logMessage("No valid QMC5883L calibration in NVM, using defaults.");
    }
    
    // Load altitude correction if it exists (relevant even if QMC fails)
    preferences.begin(PREF_NAMESPACE, true); // Re-open read-only
    altitudeCorrection = preferences.getInt(KEY_ALT_CORRECTION, 0); // Load correction, default 0
    preferences.end();

    if (altitudeCorrection != 0) {
      logMessage("Loaded Altitude Correction: " + String(altitudeCorrection) + "m");
    }
    if (compassInverted) {
      logMessage("Compass display is inverted.");
    }
  }
  else {
    logMessage("QMC5883L initialization failed, trying HMC5883L...");
    // Fall back to HMC5883L
    if (hmcCompass.begin()) {
      logMessage("HMC5883L initialized successfully.");
      activeCompass = &hmcCompass;
      
      // Load HMC declination, calibration and inversion from preferences
      preferences.begin(PREF_NAMESPACE, true); // Open NVM in read-only mode
      currentDeclination = preferences.getFloat(KEY_DECLINATION, 0.0f);
      altitudeCorrection = preferences.getInt(KEY_ALT_CORRECTION, 0); // Load correction, default 0
      bool hmcCalValid = preferences.getBool(KEY_HMC_CAL_VALID, false);
      
      // Set declination from saved value
      hmcCompass.setDeclination(currentDeclination);
      
      // Load calibration if available
      if (hmcCalValid) {
        float x_offset = preferences.getFloat(KEY_HMC_X_OFFSET, 0.0f);
        float y_offset = preferences.getFloat(KEY_HMC_Y_OFFSET, 0.0f);
        float z_offset = preferences.getFloat(KEY_HMC_Z_OFFSET, 0.0f);
        float x_scale = preferences.getFloat(KEY_HMC_X_SCALE, 1.0f);
        float y_scale = preferences.getFloat(KEY_HMC_Y_SCALE, 1.0f);
        float z_scale = preferences.getFloat(KEY_HMC_Z_SCALE, 1.0f);
        
        hmcCompass.setCalibrationOffsets(x_offset, y_offset, z_offset);
        hmcCompass.setCalibrationScales(x_scale, y_scale, z_scale);
        logMessage("Loaded HMC5883L compass calibration from NVM.");
      }
      
      preferences.end();
      
      logMessage("Using HMC5883L with declination: " + String(currentDeclination * 180.0 / PI, 2) + " degrees.");
      if (altitudeCorrection != 0) {
        logMessage("Loaded Altitude Correction: " + String(altitudeCorrection) + "m");
      }
      if (compassInverted) {
        logMessage("Compass display is inverted.");
      }
    }
    else {
      logMessage("*** WARNING: Both compass initialization attempts failed. ***");
      activeCompass = nullptr;
      // Still load altitude correction even if no compass works
      preferences.begin(PREF_NAMESPACE, true); // Re-open read-only
      altitudeCorrection = preferences.getInt(KEY_ALT_CORRECTION, 0); // Load correction, default 0
      preferences.end();
    }
  }
  
  // If we have a working compass, read initial heading
  if (activeCompass) {
    // Read raw heading directly from compass (avoid smoothing during initial read)
    int initialHeading = activeCompass->getAzimuth();
    
    // Initialize smoothed heading to actual compass heading (not 0)
    smoothedHeading = initialHeading;
    
    // Log initial compass heading
    char dirStr[4];
    activeCompass->getDirection(dirStr, initialHeading);
    logMessage("Initial compass heading: " + String(initialHeading) + "° (" + String(dirStr) + ")");
    
    // Log compass type-specific info
    if (activeCompass == &hmcCompass) {
      float declDegrees = currentDeclination * 180.0 / PI;
      logMessage("HMC5883L with declination: " + String(declDegrees, 1) + "°");
      logMessage("NOTE: HMC compass can be calibrated but requires declination setting");
    }
  }

  // In setup(), after loading other preferences:
  // ... existing code ...
  preferences.begin(PREF_NAMESPACE, true);
  useM5StackSmoothing = preferences.getBool(KEY_USE_M5_SMOOTHING, false);
  useM5StackInterference = preferences.getBool(KEY_USE_M5_INTERFERENCE, false);
  preferences.end();


  // After all initialization is complete (at the end of setup()):
  if (display_initialized) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("System Ready");
    display.println("-------------");
    if (gpsInitialized) {
      display.println("GPS: OK");
    } else {
      display.println("GPS: Error");
    }
    if (activeCompass != nullptr) {
      display.println("Compass: " + String(activeCompass->getSensorName()));
    } else {
      display.println("Compass: Not Found");
    }
    display.display();
    delay(2000);  // Show status for 2 seconds
    
    // Then switch to default display mode
    currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
    displayGraphicCompass();  // Show initial compass display
  }

  // In setup(), after loading other preferences:
  // ... existing code ...
  preferences.begin(PREF_NAMESPACE, true);
  useM5StackSmoothing = preferences.getBool(KEY_USE_M5_SMOOTHING, false);
  useM5StackInterference = preferences.getBool(KEY_USE_M5_INTERFERENCE, false);
  preferences.end();

  // After all initialization is complete (at the end of setup()):
  logMessage("Setup complete.");
  logMessage("Ready for operation.");
}

void loop() {
  // Basic hardware update - must happen first
  M5.update();

  // Simple button check with hold threshold
  M5.BtnA.setHoldThresh(800);  // Set long press threshold to 800ms
  
  UIState currentState = determineCurrentUIState();
  auto handlers = buttonHandlerMap[currentState];
  
  static bool longPressHandled = false;
  
  // Handle long press first
  if (M5.BtnA.wasHold()) {
    if (!longPressHandled && handlers.longPressHandler) {
      handlers.longPressHandler();
      longPressHandled = true;
    }
  }
  // Handle short press only if no long press occurred
  else if (M5.BtnA.wasReleased()) {
    if (!longPressHandled && handlers.shortPressHandler) {
      handlers.shortPressHandler();
    }
    longPressHandled = false;  // Reset on release
  }

  // Always read compass data if available
  if (activeCompass != nullptr) {
    activeCompass->read();
  }

  // Process any available GPS data
  if (gpsInitialized) {
    while (GPS.available()) {
      gps.encode(GPS.read());
    }
    // No need to call applyPrivacyFilter anymore - the wrapper functions handle this
  }

  // Basic display update
  if (display_initialized) {
    switch (currentDisplayMode) {
      case DisplayMode::GPS_STATUS:
        displayLogOnOLED();
        break;
      case DisplayMode::COMPASS_STATUS:
        displayCompassLogOnOLED();
        break;
      case DisplayMode::GRAPHIC_COMPASS:
        displayGraphicCompass();
        break;
      case DisplayMode::WORLD_MAP:
        displayWorldMap();
        break;
    }
  }
}

// World Map Screen
void handleShortPressWorldMap() {
  // Short press on World Map: Cycle to next display mode
  currentDisplayMode = DisplayMode::GPS_STATUS;
  logMessage("Display mode changed to: GPS_STATUS");
}

void handleLongPressWorldMap() {
  // Toggle privacy mode
  privacyModeEnabled = !privacyModeEnabled;
  logMessage("Privacy mode " + String(privacyModeEnabled ? "enabled" : "disabled"));
  
  // No need to call applyPrivacyFilter anymore since we're using the wrapper functions
  
  // Force redraw of the world map to show/hide privacy indicator
  displayWorldMap();
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

// Calibration Mode Selection
void handleShortPressCalibrationModeSelection() {
  // Short press in calibration mode selection: Rotate through options
  calibrationModeIndex = (calibrationModeIndex + 1) % NUM_CALIBRATION_MODES;
  logMessage("Calibration mode: " + String(CALIBRATION_MODE_NAMES[calibrationModeIndex]));
}

void handleLongPressCalibrationModeSelection() {
  // Long press in calibration mode selection: Select current mode
  logMessage("DEBUG: Starting calibration selection handler...");
  
  if (calibrationModeIndex == NUM_CALIBRATION_MODES - 1) {
    // Selected "Cancel" option
    isSelectingCalibrationMode = false;
    logMessage("Calibration cancelled");
  } else {
    // Selected a valid calibration mode
    calibrationPoints = CALIBRATION_MODE_POINTS[calibrationModeIndex];
    isSelectingCalibrationMode = false;
    isCalibrating = true;
    calibrationStep = 1;
    logMessage("DEBUG: Set isCalibrating=" + String(isCalibrating) + ", calibrationPoints=" + String(calibrationPoints) + ", step=" + String(calibrationStep));
    logMessage("Starting " + String(calibrationPoints) + "-point calibration...");
  }
}

// Calibrating Compass
void handleShortPressCalibrating() {
  // Short press while calibrating: Capture calibration point
  if (calibrationStep > 0 && calibrationStep <= calibrationPoints) {
    logMessage("Capturing calibration point " + String(calibrationStep) + " of " + String(calibrationPoints));
    
    if (activeCompass) {
      activeCompass->read();
      calX[calibrationStep - 1] = activeCompass->getX();
      calY[calibrationStep - 1] = activeCompass->getY();
      calibrationStep++;

      // If last point captured, calculate and apply calibration
      if (calibrationStep > calibrationPoints) {
        logMessage("Calculating " + String(calibrationPoints) + "-point calibration...");
        
        if (activeCompass->calculateCalibration(calX, calY)) {
          logMessage("Calibration calculated and applied successfully.");
          
          // Calculate noise and interference characteristics from calibration data
          float x_variance = 0.0f;
          float y_variance = 0.0f;
          float x_mean = 0.0f;
          float y_mean = 0.0f;
          
          // Calculate mean
          for (int i = 0; i < calibrationPoints; i++) {
            x_mean += calX[i];
            y_mean += calY[i];
          }
          x_mean /= calibrationPoints;
          y_mean /= calibrationPoints;
          
          // Calculate variance
          for (int i = 0; i < calibrationPoints; i++) {
            x_variance += (calX[i] - x_mean) * (calX[i] - x_mean);
            y_variance += (calY[i] - y_mean) * (calY[i] - y_mean);
          }
          x_variance /= calibrationPoints;
          y_variance /= calibrationPoints;
          
          // Set interference parameters based on calibration data
          enableOutlierRejection = true;
          enableNoiseThreshold = true;
          enableMagneticInterference = true;
          
          // Calculate thresholds based on variance
          MAX_HEADING_JUMP = (int)(sqrt(x_variance + y_variance) * 2.0f);
          HEADING_NOISE_THRESHOLD = (int)(sqrt(x_variance + y_variance) * 0.5f);
          MAGNETIC_VARIANCE_THRESHOLD = (x_variance + y_variance) * 2.0f;
          
          // Set smoothing based on noise characteristics
          ALPHA = 0.2f; // Moderate smoothing
          COMPASS_XYZ_ALPHA = 0.2f;
          
          // Save to NVM based on compass type
          preferences.begin(PREF_NAMESPACE, false);
          
          if (activeCompass == &qmcCompass) {
            // For QMC compass
            // Set calibration as valid
            preferences.putBool(KEY_CAL_VALID, true);
            
            // Since we don't have direct access to the internal calibration values,
            // we can calculate them from the min/max of our collected points
            float x_min = calX[0], x_max = calX[0];
            float y_min = calY[0], y_max = calY[0];
            
            for (int i = 1; i < calibrationPoints; i++) {
              if (calX[i] < x_min) x_min = calX[i];
              if (calX[i] > x_max) x_max = calX[i];
              if (calY[i] < y_min) y_min = calY[i];
              if (calY[i] > y_max) y_max = calY[i];
            }
            
            float x_offset = (x_max + x_min) / 2.0f;
            float y_offset = (y_max + y_min) / 2.0f;
            float z_offset = 0.0f; // We don't use Z for calibration
            
            float x_delta = (x_max - x_min) / 2.0f;
            float y_delta = (y_max - y_min) / 2.0f;
            float avg_delta = (x_delta + y_delta) / 2.0f;
            
            float x_scale = (x_delta > 0) ? avg_delta / x_delta : 1.0f;
            float y_scale = (y_delta > 0) ? avg_delta / y_delta : 1.0f;
            float z_scale = 1.0f;
            
            // Save calculated values
            preferences.putFloat(KEY_X_OFFSET, x_offset);
            preferences.putFloat(KEY_Y_OFFSET, y_offset);
            preferences.putFloat(KEY_Z_OFFSET, z_offset);
            preferences.putFloat(KEY_X_SCALE, x_scale);
            preferences.putFloat(KEY_Y_SCALE, y_scale);
            preferences.putFloat(KEY_Z_SCALE, z_scale);
            
            // Save interference mitigation settings
            preferences.putBool(KEY_OUTLIER_REJECTION, enableOutlierRejection);
            preferences.putBool(KEY_NOISE_THRESHOLD, enableNoiseThreshold);
            preferences.putBool(KEY_GPS_FUSION, enableGpsFusion);
            preferences.putBool(KEY_MAG_INTERFERENCE, enableMagneticInterference);
            preferences.putInt("maxHeadingJump", MAX_HEADING_JUMP);
            preferences.putInt("headingNoiseThresh", HEADING_NOISE_THRESHOLD);
            preferences.putFloat("magVarThresh", MAGNETIC_VARIANCE_THRESHOLD);
            preferences.putFloat("compassAlpha", ALPHA);
            preferences.putFloat("compassXYZAlpha", COMPASS_XYZ_ALPHA);
            
            logMessage("QMC5883L calibration and interference settings saved.");
          } 
          else if (activeCompass == &hmcCompass) {
            // For HMC compass
            // Same calculations as for QMC to ensure consistency
            preferences.putBool(KEY_HMC_CAL_VALID, true);
            
            float x_min = calX[0], x_max = calX[0];
            float y_min = calY[0], y_max = calY[0];
            
            for (int i = 1; i < calibrationPoints; i++) {
              if (calX[i] < x_min) x_min = calX[i];
              if (calX[i] > x_max) x_max = calX[i];
              if (calY[i] < y_min) y_min = calY[i];
              if (calY[i] > y_max) y_max = calY[i];
            }
            
            float x_offset = (x_max + x_min) / 2.0f;
            float y_offset = (y_max + y_min) / 2.0f;
            float z_offset = 0.0f;
            
            float x_delta = (x_max - x_min) / 2.0f;
            float y_delta = (y_max - y_min) / 2.0f;
            float avg_delta = (x_delta + y_delta) / 2.0f;
            
            float x_scale = (x_delta > 0) ? avg_delta / x_delta : 1.0f;
            float y_scale = (y_delta > 0) ? avg_delta / y_delta : 1.0f;
            float z_scale = 1.0f;
            
            // Save calculated values
            preferences.putFloat(KEY_HMC_X_OFFSET, x_offset);
            preferences.putFloat(KEY_HMC_Y_OFFSET, y_offset);
            preferences.putFloat(KEY_HMC_Z_OFFSET, z_offset);
            preferences.putFloat(KEY_HMC_X_SCALE, x_scale);
            preferences.putFloat(KEY_HMC_Y_SCALE, y_scale);
            preferences.putFloat(KEY_HMC_Z_SCALE, z_scale);
            
            // Save interference mitigation settings
            preferences.putBool(KEY_OUTLIER_REJECTION, enableOutlierRejection);
            preferences.putBool(KEY_NOISE_THRESHOLD, enableNoiseThreshold);
            preferences.putBool(KEY_GPS_FUSION, enableGpsFusion);
            preferences.putBool(KEY_MAG_INTERFERENCE, enableMagneticInterference);
            preferences.putInt("maxHeadingJump", MAX_HEADING_JUMP);
            preferences.putInt("headingNoiseThresh", HEADING_NOISE_THRESHOLD);
            preferences.putFloat("magVarThresh", MAGNETIC_VARIANCE_THRESHOLD);
            preferences.putFloat("compassAlpha", ALPHA);
            preferences.putFloat("compassXYZAlpha", COMPASS_XYZ_ALPHA);
            
            logMessage("HMC5883L calibration and interference settings saved.");
          }
          preferences.end();
          
          // Exit calibration mode
          isCalibrating = false;
          calibrationStep = 0;
          
          // Move to inversion setting
          isSettingInversion = true;
          
          // Log the applied interference settings
          logMessage("Applied interference settings: ");
          logMessage(" - Outlier rejection: " + String(enableOutlierRejection ? "ON" : "OFF"));
          logMessage(" - Noise threshold: " + String(enableNoiseThreshold ? "ON" : "OFF"));
          logMessage(" - GPS fusion: " + String(enableGpsFusion ? "ON" : "OFF"));
          logMessage(" - Magnetic interference: " + String(enableMagneticInterference ? "ON" : "OFF"));
          logMessage(" - Max heading jump: " + String(MAX_HEADING_JUMP) + " degrees");
          logMessage(" - Noise threshold: " + String(HEADING_NOISE_THRESHOLD) + " degrees");
          logMessage(" - Magnetic variance threshold: " + String(MAGNETIC_VARIANCE_THRESHOLD));
          logMessage(" - Compass smoothing alpha: " + String(ALPHA, 2));
          logMessage(" - Compass XYZ smoothing alpha: " + String(COMPASS_XYZ_ALPHA, 2));
        } else {
          logMessage("Calibration calculation failed. Try again.");
          isCalibrating = false;
          calibrationStep = 0;
        }
      }
    }
  }
}

void handleLongPressCalibrating() {
  // Long press while calibrating: Cancel calibration
  isCalibrating = false;
  calibrationStep = 0;
  logMessage("Calibration cancelled");
}

// Setting Declination
void handleShortPressDeclination() {
  // Short press in declination setting: Set declination
  if (activeCompass == &hmcCompass) {
    activeCompass->read();
    int magneticHeading = activeCompass->getAzimuth(); // Read the current magnetic heading

    // Calculate the declination angle
    // Declination = True North (0) - Magnetic Heading read when pointing True North
    float declinationDegrees = -magneticHeading;

    // Normalize to +/- 180 degrees
    while (declinationDegrees > 180.0f) declinationDegrees -= 360.0f;
    while (declinationDegrees <= -180.0f) declinationDegrees += 360.0f;

    // Convert to radians for storage and setting
    currentDeclination = radians(declinationDegrees);
  
    // Apply declination immediately
    hmcCompass.setDeclination(currentDeclination);
    
    // Save to NVM
    preferences.begin(PREF_NAMESPACE, false);
    preferences.putFloat(KEY_DECLINATION, currentDeclination);
    preferences.end();
    
    logMessage("Declination set to: " + String(declinationDegrees, 1) + " degrees");
    logMessage("True north should now be at compass heading: 0 deg");
    
    // Make note about calibration and declination working together
    logMessage("HMC5883L now has calibration + declination applied");
    
    // Exit declination setting
    isSettingDeclination = false;
    
    // Move to inversion setting
    isSettingInversion = true;
  }
}

void handleLongPressDeclination() {
  // Long press in declination setting: Cancel
  isSettingDeclination = false;
  logMessage("Declination setting cancelled");
}

// Setting Inversion
void handleShortPressInversion() {
  // Short press in inversion setting: Set normal mode
  compassInverted = false;
  
  // Save to NVM
  preferences.begin(PREF_NAMESPACE, false);
  preferences.putBool(KEY_COMPASS_INVERTED, false);
  preferences.end();
  
  logMessage("Compass display set to normal (non-inverted) mode");
  isSettingInversion = false;
  
  // If we're using HMC5883L and haven't set declination yet, allow it now
  if (activeCompass == &hmcCompass && !isSettingDeclination) {
    isSettingDeclination = true;
    logMessage("Now set declination for HMC5883L...");
    logMessage("Point to TRUE NORTH, then click the button.");
  }
}

void handleLongPressInversion() {
  // Long press in inversion setting: Set inverted mode
  compassInverted = true;
  
  // Save to NVM
  preferences.begin(PREF_NAMESPACE, false);
  preferences.putBool(KEY_COMPASS_INVERTED, true);
  preferences.end();
  
  logMessage("Compass display set to inverted mode");
  isSettingInversion = false;
}

// Button handler implementations
void handleShortPressGPSStatus() {
  // Switch to next screen
  currentDisplayMode = DisplayMode::COMPASS_STATUS;
  logMessage("Display mode changed to: COMPASS_STATUS");
}

void handleLongPressGPSStatus() {
  // Toggle privacy mode
  privacyModeEnabled = !privacyModeEnabled;
  preferences.putBool(KEY_COMPASS_INVERTED, privacyModeEnabled);
  
  // Immediately apply privacy filter to update displayed coordinates
  if (gps.location.isValid()) {
    applyPrivacyFilter();
  }
  
  // Clearly log that this is GPS coordinate privacy
  if (privacyModeEnabled) {
    logMessage("GPS PRIVACY MODE ENABLED - Coordinates will be masked");
  } else {
    logMessage("GPS PRIVACY MODE DISABLED - Showing actual coordinates");
  }
}

void handleShortPressCompassStatus() {
  // Switch to next screen
  currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
  logMessage("Display mode changed to: GRAPHIC_COMPASS");
}

void handleLongPressCompassStatus() {
  // Start calibration mode selection
  if (activeCompass == &qmcCompass) {
    isSelectingCalibrationMode = true;
    calibrationModeIndex = 1; // Default to 8-point
    logMessage("Entering calibration mode selection...");
  } else if (activeCompass == &hmcCompass) {
    isSettingDeclination = true;
    logMessage("Entering declination setting mode...");
  }
}

void handleShortPressGraphicCompass() {
  // Switch to next screen
  currentDisplayMode = DisplayMode::WORLD_MAP;
  logMessage("Display mode changed to: WORLD_MAP");
}

void handleLongPressGraphicCompass() {
  // Toggle compass inversion
  compassInverted = !compassInverted;
  logMessage("Compass display " + String(compassInverted ? "inverted" : "normal"));
}

void scanI2CDevices() {
  byte error, address;
  int nDevices = 0;
  
  logMessage("Scanning I2C bus...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      String msg = "I2C device found at address 0x";
      if (address < 16) msg += "0";
      msg += String(address, HEX);
      logMessage(msg);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    logMessage("No I2C devices found");
  }
}

// Display function implementations
void updateDisplay(float lat, float lng, float alt, int heading) {
  if (!display_initialized) return;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("GPS Data:");
  display.println("-------------");
  
  // Format and display coordinates
  display.print("Lat: "); display.println(lat, 6);
  display.print("Lng: "); display.println(lng, 6);
  display.print("Alt: "); display.print(alt + altitudeCorrection); display.println("m");
  
  // --- Compass Direction and Azimuth --- 
  char dirArray[4] = {' ', ' ', ' ', '\0'}; // Array to hold direction text (e.g., NNE)
  
  if (activeCompass != nullptr) {
    int displayHeading = heading;
    // Apply inversion if compass is inverted
    if (compassInverted) {
      displayHeading = (heading + 180) % 360;
    }
    
    activeCompass->getDirection(dirArray, displayHeading);
    
    display.setCursor(0, 40); // Position for Direction Text
    display.print(dirArray);
    
    display.setCursor(0, 48); // Position for Azimuth value
    display.print("Az ");
    display.println(displayHeading);
    
    // Show declination if using HMC5883L
    if (activeCompass == &hmcCompass) {
      float declDegrees = currentDeclination * 180.0 / PI;
      display.setCursor(50, 40);
      display.print("Decl:");
      display.print(declDegrees, 1);
    }
  } else {
    display.setCursor(0, 40);
    display.print("No compass");
    display.setCursor(0, 48);
    display.print("Az: ---");
  }
  
  // Display satellite count
  display.setCursor(0, 56);
  display.print("Sats: ");
  display.println(gps.satellites.value());
  
  display.display();
}

void displayLogOnOLED() {
  if (!display_initialized) return;

  // Use static variables to hold the protocol string and its last update time
  static String displayedGpsProtocol = "Initializing...";
  static unsigned long lastProtocolUpdate = 0;
  const unsigned long PROTOCOL_UPDATE_INTERVAL = 5000; // 5 seconds

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

  // Calculate packets per second
  unsigned long currentPackets = gps.passedChecksum();
  unsigned long currentTime = millis();
  if (currentTime - lastPacketTime >= 1000) { // Update rate every second
      packetsPerSecond = (currentPackets - lastPacketCount) * 1000.0f / (currentTime - lastPacketTime);
      lastPacketCount = currentPackets;
      lastPacketTime = currentTime;
  }

  display.print("Pkt/s ");
  display.print(packetsPerSecond, 1);
  display.print(" Err=");
  display.println(gps.failedChecksum());

  // Display course and speed
  if (gps.course.isValid() && gps.speed.isValid()) {
    display.setCursor(0, 52); // Adjusted Y position
    display.print("Course: ");
    display.print(gps.course.deg());
    display.println(" deg");
    
    display.setCursor(64, 52); // Adjusted X position for speed
    display.print("Spd: ");
    float speedKmph = gps.speed.kmph();
    bool reliableSpeed = gps.satellites.value() >= 4 && gps.hdop.isValid() && gps.hdop.hdop() < 3.0;
    if (!reliableSpeed || speedKmph < 3.0) speedKmph = 0;
    display.print(speedKmph, 1);
    // Removed "km/h" to save space
  }

  // Draw privacy indicator if privacy mode is enabled
  if (privacyModeEnabled) {
    drawPrivacyIndicator(display);
  }
  
  display.display();
}

void displayCompassLogOnOLED() {
  if (!display_initialized) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (isSelectingCalibrationMode) {
    display.setCursor(0, 0);
    display.println("Select Cal Mode:");
    display.println("---------------");
    
    for (int i = 0; i < NUM_CALIBRATION_MODES; i++) {
      if (i == calibrationModeIndex) {
        display.print("> ");
      } else {
        display.print("  ");
      }
      display.println(CALIBRATION_MODE_NAMES[i]);
    }
    
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.println("Click: Next Option");
    display.println("Hold: Select Option");
  }
  else if (isCalibrating && activeCompass == &qmcCompass) {
    display.setCursor(0, 0);
    display.println("-- Calibrating --");
    display.setCursor(0, 10);
    display.print("Mode: ");
    display.print(calibrationPoints);
    display.println("-point");
    
    display.setCursor(0, 26);
    String directions = "";
    
    if (calibrationPoints == 4) {
      switch (calibrationStep) {
        case 1: directions = "Point NORTH, Click"; break;
        case 2: directions = "Point EAST, Click"; break;
        case 3: directions = "Point SOUTH, Click"; break;
        case 4: directions = "Point WEST, Click"; break;
      }
    } 
    else if (calibrationPoints == 8) {
      switch (calibrationStep) {
        case 1: directions = "Point NORTH, Click"; break;
        case 2: directions = "Point SOUTH, Click"; break;
        case 3: directions = "Point EAST, Click"; break;
        case 4: directions = "Point WEST, Click"; break;
        case 5: directions = "Point NE, Click"; break;
        case 6: directions = "Point SE, Click"; break;
        case 7: directions = "Point SW, Click"; break;
        case 8: directions = "Point NW, Click"; break;
      }
    } 
    else {
      directions = "Point to " + String(calibrationStep * 22.5) + " deg";
    }
    
    display.println(directions);
    
    display.setCursor(0, 46);
    display.print("Progress: ");
    display.print(calibrationStep - 1);
    display.print("/");
    display.println(calibrationPoints);
    
    display.setCursor(0, 56);
    display.println("(Hold Btn to Cancel)");
  }
  else if (isSettingDeclination && activeCompass == &hmcCompass) {
    display.setCursor(0, 0);
    display.println("- Set Declination -");
    
    display.setCursor(0, 16);
    display.println("Point device to TRUE");
    display.setCursor(0, 26);
    display.println("NORTH, then click");
    
    display.setCursor(0, 40);
    display.print("Current: ");
    display.print(activeCompass->getAzimuth());
    display.println(" deg");
    
    display.setCursor(0, 56);
    display.println("Hold: Cancel");
  }
  else if (isSettingInversion) {
    display.setCursor(0, 0);
    display.println("Invert Compass?");
    display.println("---------------");
    
    int centerX = SCREEN_WIDTH / 2;
    int centerY = 30;
    int radius = 15;
    
    if (activeCompass != nullptr) {
      int heading = activeCompass->getAzimuth();
      if (compassInverted) {
        heading = (heading + 180) % 360;
      }
      
      float angleRad = radians(270 - heading);
      int endX = centerX + radius * cos(angleRad);
      int endY = centerY + radius * sin(angleRad);
      
      display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
      display.drawLine(centerX, centerY, endX, endY, SSD1306_WHITE);
      display.fillCircle(centerX, centerY, 2, SSD1306_WHITE);
      
      display.setCursor(centerX - 2, centerY - radius - 6);
      display.print("N");
      display.setCursor(centerX - 2, centerY + radius);
      display.print("S");
      display.setCursor(centerX + radius, centerY - 2);
      display.print("E");
      display.setCursor(centerX - radius - 5, centerY - 2);
      display.print("W");
      
      display.setCursor(68, 14);
      display.print("Hdg:");
      display.print(heading);
    }
    
    display.setCursor(0, SCREEN_HEIGHT - 18);
    display.println("Click: Normal");
    display.setCursor(70, SCREEN_HEIGHT - 18);
    display.println("Hold: Invert");
    
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print("Current: ");
    display.print(compassInverted ? "Inverted" : "Normal");
  }
  else {
    // Normal Compass Status Screen
    String title = "-- Compass Status --";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println(title);
    
    if (activeCompass != nullptr) {
      // Display compass type on first line
      display.setCursor(0, 12);
      display.print(activeCompass->getSensorName());
      
      // Show declination if using HMC5883L on same line
      if (activeCompass == &hmcCompass) {
        float declDegrees = currentDeclination * 180.0 / PI;
        display.print(" Decl:");
        display.print(declDegrees, 1);
      }
      
      // Display heading on next line
      activeCompass->read();
      int displayHeading = activeCompass->getAzimuth();
      if (compassInverted) {
        displayHeading = (displayHeading + 180) % 360;
      }
      
      // Move heading and direction to next line
      display.setCursor(0, 24);
      display.print("Az ");
      display.print(displayHeading);
      display.print(" deg ");
      
      char dirArray[4] = {' ', ' ', ' ', '\0'};
      activeCompass->getDirection(dirArray, displayHeading);
      display.print(dirArray);
      
      // Raw sensor data with proper spacing
      display.setCursor(0, 36);
      display.print("X: ");
      display.println(activeCompass->getX());
      
      display.setCursor(0, 44);
      display.print("Y: ");
      display.println(activeCompass->getY());
      
      display.setCursor(0, 52);
      display.print("Z: ");
      display.println(activeCompass->getZ());
      
      // Show inversion status at bottom
      if (compassInverted) {
        String invStr = "[Inverted]";
        display.getTextBounds(invStr, 0, 0, &x1, &y1, &w, &h);
        display.setCursor(0, SCREEN_HEIGHT - 8);
        display.print(invStr);
      }
    } else {
      display.setCursor(0, 24);
      display.println("No compass detected");
    }
    
    // Add calibration prompt in bottom right if compass is active
    if (activeCompass != nullptr) {
      String calibPrompt = "";
      if (activeCompass == &qmcCompass) {
        calibPrompt = "Cal: Hold";
      } else if (activeCompass == &hmcCompass) {
        calibPrompt = "Decl: Hold";
      }
      
      // Right align the calibration prompt
      display.getTextBounds(calibPrompt, 0, 0, &x1, &y1, &w, &h);
      display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 8);
      display.print(calibPrompt);
    }
  }

  display.display();
}

void displayGraphicCompass() {
  if (!display_initialized) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2;
  int radius = 22;
  int heading = 0;
  
  if (activeCompass != nullptr) {
    heading = activeCompass->getAzimuth();
    if (compassInverted) {
      heading = (heading + 180) % 360;
    }
    
    float angleRad = radians(270 - heading);
    
    display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
    
    int endX = centerX + radius * cos(angleRad);
    int endY = centerY + radius * sin(angleRad);
    display.drawLine(centerX, centerY, endX, endY, SSD1306_WHITE);
    
    float arrowAngle1 = angleRad + radians(150);
    float arrowAngle2 = angleRad + radians(210);
    int arrowLength = 6;
    int arrow1X = endX + arrowLength * cos(arrowAngle1);
    int arrow1Y = endY + arrowLength * sin(arrowAngle1);
    int arrow2X = endX + arrowLength * cos(arrowAngle2);
    int arrow2Y = endY + arrowLength * sin(arrowAngle2);
    display.drawLine(endX, endY, arrow1X, arrow1Y, SSD1306_WHITE);
    display.drawLine(endX, endY, arrow2X, arrow2Y, SSD1306_WHITE);
    
    display.fillCircle(centerX, centerY, 2, SSD1306_WHITE);
    
    display.setCursor(centerX - 2, centerY - radius - 7);
    display.print("N");
    display.setCursor(centerX - 2, centerY + radius + 1);
    display.print("S");
    display.setCursor(centerX + radius + 2, centerY - 3);
    display.print("E");
    display.setCursor(centerX - radius - 7, centerY - 3);
    display.print("W");
  } else {
    display.setCursor(centerX - 8, centerY - 4);
    display.print("ERR");
  }

  if (activeCompass != nullptr) {
    display.setCursor(0, 0);
    display.print("Az ");
    display.print(heading);
  } else {
    display.setCursor(0, 0);
    display.print("Az ---");
  }
  
  if (gps.altitude.isValid()) {
    bool reliableAlt = gps.satellites.value() >= 5 && gps.hdop.isValid() && gps.hdop.hdop() < 2.5;
    float correctedAlt = gps.altitude.meters() + altitudeCorrection;
    
    if (reliableAlt) {
      String altStr = "Alt " + String(correctedAlt, 0) + "m";
      int16_t x1, y1;
      uint16_t w, h;
      display.getTextBounds(altStr, 0, 0, &x1, &y1, &w, &h);
      display.setCursor(SCREEN_WIDTH - w, 0);
      display.print(altStr);
    }
  }
  
  if (gps.speed.isValid()) {
    bool reliableSpeed = gps.satellites.value() >= 4 && gps.hdop.isValid() && gps.hdop.hdop() < 3.0;
    float speedKmph = gps.speed.kmph();
    if (!reliableSpeed || speedKmph < 3.0) speedKmph = 0;
    
    if (speedKmph > 0) {
      String spdStr = "Spd " + String(speedKmph, 1) + "km/h";
      int16_t x1, y1;
      uint16_t w, h;
      display.getTextBounds(spdStr, 0, 0, &x1, &y1, &w, &h);
      display.setCursor(centerX + radius + 4, centerY - h/2);
      display.print(spdStr);
    }
  }
  
  if (gps.location.isValid()) {
    // Use privacy-filtered coordinates via wrapper functions
    float lat = getLatitude(); 
    float lng = getLongitude();
    
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.println("Lat");
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print(String(lat, 5));
    
    String lngLabel = "Lng";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(lngLabel, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 16);
    display.println(lngLabel);
    
    String lngValue = String(lng, 5);
    display.getTextBounds(lngValue, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 8);
    display.print(lngValue);
  } else {
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.println("Lat");
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.println("No Fix");
    
    String lngLabel = "Lng";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(lngLabel, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 16);
    display.println(lngLabel);
    
    String noFix = "No Fix";
    display.getTextBounds(noFix, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 8);
    display.println(noFix);
  }
  
  // Draw privacy indicator if privacy mode is enabled
  if (privacyModeEnabled) {
    drawPrivacyIndicator(display);
  }

  display.display();
}

void displayWorldMap() {
  if (!display_initialized) return;

  display.clearDisplay();
  
  display.drawBitmap(0, 0, world_map, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  
  if (gps.location.isValid()) {
    // Use the wrapper functions that handle privacy mode automatically
    float lat = getLatitude();
    float lng = getLongitude();
    
    int posX = (int)((lng + 180.0) / 360.0 * SCREEN_WIDTH);
    
    if (lat > 85.0) lat = 85.0;
    if (lat < -85.0) lat = -85.0;
    
    float latRad = lat * PI / 180.0;
    float mercN = log(tan((PI/4) + (latRad/2)));
    int posY = (int)(SCREEN_HEIGHT/2 - (mercN * SCREEN_HEIGHT / (2*PI)));
    
    posX = constrain(posX, 0, SCREEN_WIDTH-1);
    posY = constrain(posY, 0, SCREEN_HEIGHT-1);
    
    if ((millis() / 500) % 2 == 0) {
      display.fillCircle(posX, posY, 3, SSD1306_WHITE);
      display.drawCircle(posX, posY, 4, SSD1306_WHITE);
    } else {
      display.drawCircle(posX, posY, 3, SSD1306_WHITE);
      display.drawCircle(posX, posY, 4, SSD1306_WHITE);
    }
    
    if (activeCompass != nullptr) {
      activeCompass->read();
      int heading = activeCompass->getAzimuth();
      
      if (compassInverted) {
        heading = (heading + 180) % 360;
      }
      
      float radians = heading * PI / 180.0;
      int arrowLength = 8;
      int endX = posX + sin(radians) * arrowLength;
      int endY = posY - cos(radians) * arrowLength;
      
      display.drawLine(posX, posY, endX, endY, SSD1306_WHITE);
    }
  } else {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(2, SCREEN_HEIGHT - 8);
    display.print("No Fix");
  }
  
  // Draw privacy indicator (lock icon) when privacy mode is enabled
  if (privacyModeEnabled) {
    drawPrivacyIndicator(display);
  }
  
  display.display();
}
 
 