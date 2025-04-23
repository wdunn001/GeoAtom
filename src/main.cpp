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

// Replace old compass setup with interface pointer 
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
bool privacyModeEnabled = false; // Flag to toggle privacy mode for coordinates

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

// Pin definitions based on physical connections
// --- Verified Working Configuration for BN-880 & M5Atom Echo ---
// I2C pins - shared between compass (BN-880) and display (SSD1306)
#define I2C_SDA 26 // Grove Pin 1
#define I2C_SCL 32 // Grove Pin 2

// GPS module connections (BN-880 UART)
#define GPS_RX 25  // ESP32 RX pin connected to BN-880 TX
#define GPS_TX 21  // ESP32 TX pin connected to BN-880 RX

// TTL to RS232 level converter connections (Optional Host Output)
#define HOST_RX 33  // ESP32 RX pin connected to TTL TX
#define HOST_TX 23  // ESP32 TX pin connected to TTL RX
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
const float ALPHA = 0.15f;     // Smoothing factor (0.0-1.0): lower = more smoothing, higher = more responsive
// Compass raw value smoothing
int smoothedCompassX = 0;      // Smoothed compass X value
int smoothedCompassY = 0;      // Smoothed compass Y value 
int smoothedCompassZ = 0;      // Smoothed compass Z value
const float COMPASS_XYZ_ALPHA = 0.2f; // Smoothing factor for raw compass values
bool compassValuesInitialized = false; // Flag to initialize compass values
bool compassStabilized = false; // Flag to indicate compass has stabilized after startup
unsigned long compassStartupTime = 0; // Timestamp when compass initialization started
const unsigned long COMPASS_STABILIZATION_TIME = 3000; // Wait 3 seconds for compass to stabilize

// Privacy mode variables
float privacyLat = 0.0f;  // Latitude after privacy filter is applied
float privacyLng = 0.0f;  // Longitude after privacy filter is applied

// Function to apply privacy filter to GPS coordinates
void applyPrivacyFilter() {
  // Only process if we have valid GPS data
  if (gps.location.isValid()) {
    float rawLat = gps.location.lat();
    float rawLng = gps.location.lng();
    
    if (privacyModeEnabled) {
      // Privacy mode: Transform coordinates to only keep first digit
      
      // Process latitude
      float absLat = abs(rawLat);
      int latInt = static_cast<int>(absLat);
      String latStr = String(latInt);
      
      if (latInt == 0) {
        // If latitude is less than 1, set to 0
        privacyLat = (rawLat < 0) ? -0.0f : 0.0f;
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
        privacyLat = (rawLat < 0) ? -maskedLat : maskedLat;
      }
      
      // Process longitude
      float absLng = abs(rawLng);
      int lngInt = static_cast<int>(absLng);
      String lngStr = String(lngInt);
      
      if (lngInt == 0) {
        // If longitude is less than 1, set to 0
        privacyLng = (rawLng < 0) ? -0.0f : 0.0f;
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
        privacyLng = (rawLng < 0) ? -maskedLng : maskedLng;
      }
      
      // Log privacy transformation (occasionally)
      static unsigned long lastPrivacyLog = 0;
      if (millis() - lastPrivacyLog > 30000) { // Log every 30 seconds
        lastPrivacyLog = millis();
        logMessage("Privacy filter: " + String(rawLat, 6) + " -> " + String(privacyLat, 6) + 
                  ", " + String(rawLng, 6) + " -> " + String(privacyLng, 6));
      }
    } else {
      // No privacy mode: Use raw GPS data
      privacyLat = rawLat;
      privacyLng = rawLng;
    }
  } else {
    // Invalid GPS: Set both to zero
    privacyLat = 0.0f;
    privacyLng = 0.0f;
  }
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
// Configuration flags - these can be toggled in settings
bool enableOutlierRejection = true;      // Reject sudden large jumps in heading
bool enableNoiseThreshold = true;        // Ignore small changes below threshold
bool enableGpsFusion = true;             // Blend GPS course when moving
bool enableMagneticInterference = true;  // Detect and compensate for interference

// Constants for interference mitigation
const int MAX_HEADING_JUMP = 60;           // Increased from 30 to 60 degrees - Maximum allowed heading change in degrees per reading
const int HEADING_NOISE_THRESHOLD = 3;     // Degrees - changes smaller than this are ignored
const float MIN_GPS_CONFIDENCE = 0.7;      // Minimum confidence to start using GPS course
const float MAGNETIC_VARIANCE_THRESHOLD = 2000.0; // Threshold for magnetic field variance indicating interference

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

// Forward declarations for handler functions
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

// Define a type for button handler functions for clarity
typedef void (*ButtonHandlerFunc)();

// Define a struct to hold both short and long press handlers
struct ButtonHandlers {
    ButtonHandlerFunc shortPressHandler;
    ButtonHandlerFunc longPressHandler;
};

// Create a map from UIState to button handlers
std::map<UIState, ButtonHandlers> buttonHandlerMap;

void setup() {
  delay(1000);  // Give peripherals time to initialize
  
  // Initialize the button handler map
  buttonHandlerMap[UIState::GPS_STATUS_SCREEN] = {handleShortPressGPSStatus, handleLongPressGPSStatus};
  buttonHandlerMap[UIState::COMPASS_STATUS_SCREEN] = {handleShortPressCompassStatus, handleLongPressCompassStatus};
  buttonHandlerMap[UIState::GRAPHIC_COMPASS_SCREEN] = {handleShortPressGraphicCompass, handleLongPressGraphicCompass};
  buttonHandlerMap[UIState::WORLD_MAP_SCREEN] = {handleShortPressWorldMap, handleLongPressWorldMap};
  buttonHandlerMap[UIState::ALTITUDE_CORRECTION_MODE] = {handleShortPressAltitudeCorrection, handleLongPressAltitudeCorrection};
  buttonHandlerMap[UIState::CALIBRATION_MODE_SELECTION] = {handleShortPressCalibrationModeSelection, handleLongPressCalibrationModeSelection};
  buttonHandlerMap[UIState::CALIBRATING_COMPASS] = {handleShortPressCalibrating, handleLongPressCalibrating};
  buttonHandlerMap[UIState::SETTING_DECLINATION] = {handleShortPressDeclination, handleLongPressDeclination};
  buttonHandlerMap[UIState::SETTING_INVERSION] = {handleShortPressInversion, handleLongPressInversion};
  
  // Initialize I2C with appropriate frequency for reliable operation
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // 100kHz is more reliable than the default 400kHz
  
  // Initialize M5 hardware with custom config for Atom Echo
  auto cfg = M5.config();
  cfg.serial_baudrate = 0;  // Disable serial init in M5Unified completely
  M5.begin(cfg);
  
  // Initialize serial communications
  // --- Start with the default/known working baud rate for initial communication ---
  gpsBaudRate = 9600; // Store GPS baud rate
  GPS.begin(gpsBaudRate, SERIAL_8N1, GPS_RX, GPS_TX); 
  Host.begin(115200, SERIAL_8N1, HOST_RX, HOST_TX);
  
  // Output message on the host serial - now using logMessage
  logMessage("\n\n--- GPS Navigation System ---");
  logMessage("System initializing...");

  // --- Configure GPS Module --- 
  logMessage("Configuring GPS module...");
  delay(300); // Wait for GPS module to be ready for commands
  GPSConfigurator gpsConfig(GPS);

  // First set baud rate to 9600
  if (gpsConfig.setBaudRate(9600)) {
      logMessage("GPS: Set baud rate to 9600.");
  } else {
      logMessage("GPS: Failed to set baud rate to 9600.");
  }
  delay(100); // Small delay between commands

  // Set Update Rate to 10Hz
  if (gpsConfig.setUpdateRateHz(10)) {
      logMessage("GPS: Set update rate to 10 Hz.");
  } else {
      logMessage("GPS: Failed to set update rate to 10 Hz.");
  }
  delay(100); // Small delay between commands

  // Save configuration to make it persistent
  if (gpsConfig.saveConfiguration()) {
      logMessage("GPS: Configuration saved to NVM.");
  } else {
      logMessage("GPS: Failed to save configuration.");
  }
  delay(100);
  logMessage("GPS configuration complete.");
  // --- End GPS Configuration ---
  
  // Scan I2C devices to identify what's connected
  scanI2CDevices(); // Keep original Host prints inside this function for now
  
  // --- Initialize Compass with Failover --- 
  logMessage("Initializing compass (attempt 1: QMC5883L)... ");
  
  // Try QMC5883L first
  if (qmcCompass.begin()) {
      logMessage("QMC5883L initialized successfully.");
      activeCompass = &qmcCompass;
      
      // Load QMC calibration from preferences
      preferences.begin(PREF_NAMESPACE, true); // Open NVM in read-only mode
      bool calValid = preferences.getBool(KEY_CAL_VALID, false);
      compassInverted = preferences.getBool(KEY_COMPASS_INVERTED, false);
      
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
          compassInverted = preferences.getBool(KEY_COMPASS_INVERTED, false);
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
            if (altitudeCorrection != 0) {
              logMessage("Loaded Altitude Correction: " + String(altitudeCorrection) + "m");
          }
      }
  }
  
  logMessage("Compass setup complete."); 
  
  // Initialize SSD1306 OLED display with better error handling
  logMessage("Initializing display..."); // Use logMessage
  
  // Try multiple initialization attempts - in case of power-up issues
  for (int attempt = 0; attempt < 3; attempt++) {
    // Try the most common address (0x3C)
    if(display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      logMessage("Display initialized at address 0x3C"); // Use logMessage
      display_initialized = true;
      break;
    } 
    // Try alternative address (0x3D)
    else if(display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      logMessage("Display initialized at address 0x3D"); // Use logMessage
      display_initialized = true;
      break;
    }
    else {
      // Keep original Host.print/println for this dynamic message
      Host.print("Display init attempt "); 
      Host.print(attempt + 1);
      Host.println(" failed, retrying...");
      delay(100);  // Short delay before retry
    }
  }
  
  if (!display_initialized) {
    logMessage("*** WARNING: Could not initialize SSD1306 display"); // Use logMessage
    logMessage("*** Check connections and I2C address"); // Use logMessage
  }
  
  // If display initialized, show welcome message (No logging needed here, it's on OLED)
  if (display_initialized) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("GPS Navigation");
    display.println("-------------");
    display.println("Initializing...");
    display.println("Waiting for GPS");
    display.display();
  }
  
  // Send final setup information - use logMessage
  logMessage("\nSystem initialization complete");
  logMessage("I2C: SDA=26, SCL=32 (Grove Port A)");
  logMessage("GPS (BN-880 UART): RX=25, TX=21 @ " + String(gpsBaudRate));
  logMessage("Host (TTL UART): RX=33, TX=23 @ 115200");
  logMessage("Waiting for GPS data...");
  
  // Load interference mitigation settings
  preferences.begin(PREF_NAMESPACE, true); // Open read-only
  enableOutlierRejection = preferences.getBool(KEY_OUTLIER_REJECTION, true);
  enableNoiseThreshold = preferences.getBool(KEY_NOISE_THRESHOLD, true);
  enableGpsFusion = preferences.getBool(KEY_GPS_FUSION, true);
  enableMagneticInterference = preferences.getBool(KEY_MAG_INTERFERENCE, true);
  preferences.end();
  
  logMessage("Compass interference mitigation settings:");
  logMessage(" - Outlier rejection: " + String(enableOutlierRejection ? "ON" : "OFF"));
  logMessage(" - Noise threshold: " + String(enableNoiseThreshold ? "ON" : "OFF"));
  logMessage(" - GPS fusion: " + String(enableGpsFusion ? "ON" : "OFF"));
  logMessage(" - Magnetic interference: " + String(enableMagneticInterference ? "ON" : "OFF"));
  
  if (activeCompass != nullptr) {
    // Take initial readings to stabilize compass
    logMessage("Initializing compass readings...");
    compassStartupTime = millis();
    
    // Take multiple readings to stabilize the sensor
    for (int i = 0; i < 10; i++) {
      activeCompass->read();
      delay(50);
    }
    
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
}

void loop() {
  // Update M5 hardware state (buttons, etc.)
  M5.update();

  // Normal GPS data processing
  while (GPS.available() > 0) {
    char c = GPS.read();
    gps.encode(c);
  }

  // Apply privacy filter if GPS data was updated
  if (gps.location.isValid()) {
    applyPrivacyFilter();
  }
}

// World Map Screen
void handleShortPressWorldMap() {
  // Short press on World Map: Cycle to next display mode
  currentDisplayMode = DisplayMode::GPS_STATUS;
  logMessage("Display mode changed to: GPS_STATUS");
}

void handleLongPressWorldMap() {
  // Long press on World Map: Toggle privacy mode
  privacyModeEnabled = !privacyModeEnabled;
  
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
            
            // Also save interference mitigation settings
            preferences.putBool(KEY_OUTLIER_REJECTION, enableOutlierRejection);
            preferences.putBool(KEY_NOISE_THRESHOLD, enableNoiseThreshold);
            preferences.putBool(KEY_GPS_FUSION, enableGpsFusion);
            preferences.putBool(KEY_MAG_INTERFERENCE, enableMagneticInterference);
            
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
            
            // Also save interference mitigation settings (these apply to both compass types)
            preferences.putBool(KEY_OUTLIER_REJECTION, enableOutlierRejection);
            preferences.putBool(KEY_NOISE_THRESHOLD, enableNoiseThreshold);
            preferences.putBool(KEY_GPS_FUSION, enableGpsFusion);
            preferences.putBool(KEY_MAG_INTERFERENCE, enableMagneticInterference);
            
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
    int magneticHeading = activeCompass->getAzimuth();
    
    // Convert heading to the -180 to +180 format for declination
    float declination = magneticHeading;
    if (declination > 180) declination -= 360;
    
    // Convert to radians
    currentDeclination = radians(declination);
  
    // Apply declination immediately
    hmcCompass.setDeclination(currentDeclination);
    
    // Save to NVM
    preferences.begin(PREF_NAMESPACE, false);
    preferences.putFloat(KEY_DECLINATION, currentDeclination);
    preferences.end();
    
    float declDegrees = currentDeclination * 180.0 / PI;
    logMessage("Declination set to: " + String(declDegrees, 1) + " degrees");
    logMessage("True north should now be at compass heading: 0°");
    
    // Make note about calibration and declination working together
    logMessage("HMC5883L now has both calibration + declination applied");
    logMessage("This gives best accuracy: Calibration fixes distortion, declination aligns to true north");
    
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
 