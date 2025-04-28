#include <Arduino.h>
#include <HardwareSerial.h>
#include <M5Unified.h>
#include <TinyGPS++.h>
#include "CompassInterface.h"
#include <vector>
#include <String>
#include <Preferences.h>
#include "GPS_Configurator.h"
#include "ICOM7100Configurator.h"  // Add new include
#include <map> // Add for std::map
// Include SSD1306 display libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <algorithm>  // Add this for min function

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

void scanI2CDevices(void);
void displayLogOnOLED();
void displayCompassLogOnOLED();
void displayGraphicCompass();
void displayWorldMap();
void displayLogMessages();
void displayRadioSettings();
void displayRadioStatus();
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

// Add this function to draw a save icon
void drawSaveIcon(Adafruit_SSD1306 &display) {
  // Draw a floppy disk icon at top-right corner
  int x = display.width() - 10;
  int y = 2;
  
  // Disk body
  display.fillRect(x, y, 8, 8, SSD1306_WHITE);
  
  // Disk label
  display.drawRect(x + 2, y + 2, 4, 4, SSD1306_BLACK);
  
  // Disk shutter
  display.drawRect(x - 1, y + 1, 2, 6, SSD1306_WHITE);
}

// GPS setup
TinyGPSPlus gps;
HardwareSerial GPS(1);  // Using UART1 for GPS
HardwareSerial Radio(2); // Using UART2 for NMEA forwarding to Radio/TTL converter
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
enum class DisplayMode { GPS_STATUS, COMPASS_STATUS, GRAPHIC_COMPASS, WORLD_MAP, LOG_DISPLAY, RADIO_SETTINGS, RADIO_STATUS };
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

// TTL to RS232 level converter connections (Optional Radio Output)
#define RADIO_RX 22  // ESP32 RX pin connected to TTL TX
#define RADIO_TX 19  // ESP32 TX pin connected to TTL RX
// --------------------------------------------------------------

// Create a global log instance
m5::Log_Class m5Log;

// Create ICOM7100 configurator instance
ICOM7100Configurator* radioConfig = nullptr;

// Logging function - adds messages to buffer and sends to M5 log
void logMessage(const String& msg) {
  // Log to M5 serial (USB) for debugging
  m5Log.println(msg.c_str());
  
  // Only add to buffer if it's an error, warning, or failure message
  if (msg.startsWith("ERROR") || 
      msg.startsWith("***") || 
      msg.startsWith("WARNING") ||
      msg.startsWith("Failed") ||
      msg.startsWith("Error") ||
      msg.startsWith("!") ||
      msg.startsWith("Warning")) {
    
    // Add message to buffer, managing size
    if (log_buffer.size() >= MAX_LOG_LINES) {
      log_buffer.erase(log_buffer.begin()); // Remove the oldest message
    }
    log_buffer.push_back(msg); // Add the new message
  }
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

// UI State Management - Centralized Button Handling
// Define all possible UI states
enum class UIState {
  // Main display modes
  GPS_STATUS_SCREEN,
  COMPASS_STATUS_SCREEN,
  GRAPHIC_COMPASS_SCREEN,
  WORLD_MAP_SCREEN,
  RADIO_SETTINGS_SCREEN,
  RADIO_STATUS_SCREEN,
  
  // Configuration modes
  ALTITUDE_CORRECTION_MODE,
  CALIBRATION_MODE_SELECTION,
  CALIBRATING_COMPASS,
  SETTING_DECLINATION,
  SETTING_INVERSION,
  LOG_DISPLAY
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
      case DisplayMode::LOG_DISPLAY:
        detectedState = UIState::LOG_DISPLAY;
        break;
      case DisplayMode::RADIO_SETTINGS:
        detectedState = UIState::RADIO_SETTINGS_SCREEN;
        break;
      case DisplayMode::RADIO_STATUS:
        detectedState = UIState::RADIO_STATUS_SCREEN;
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
void handleShortPressLogDisplay();
void handleLongPressLogDisplay();
void handleShortPressRadioSettings();
void handleLongPressRadioSettings();
void handleShortPressRadioStatus();
void handleLongPressRadioStatus();
void handleDoubleClickRadioSettings();
void handleDoubleClickCompassStatus();
// Function prototypes
void scanI2CDevices();
void forwardNMEAToICOM(TinyGPSPlus& gps, int altitudeCorrection);


// Define a type for button handler functions for clarity
typedef void (*ButtonHandlerFunc)();

// Define a struct to hold both short and long press handlers
struct ButtonHandlers {
    ButtonHandlerFunc shortPressHandler;
    ButtonHandlerFunc longPressHandler;
    ButtonHandlerFunc doubleClickHandler;
};

// Create a map from UIState to button handlers
std::map<UIState, ButtonHandlers> buttonHandlerMap;

// Add these declarations near the top with other variables
const unsigned long GPS_CONFIG_RETRY_DELAY = 100; // 100ms between retries
bool gpsInitialized = false;  // Track if GPS was successfully initialized

// Add these variables near the top with other global variables
static unsigned long lastPacketCount = 0;
static unsigned long lastPacketTime = 0;
static float packetsPerSecond = 0.0f;


// Add these variables near the other global variables
static int currentLogIndex = 0;  // Index of the current message being displayed
static const int MESSAGES_PER_PAGE = 1; // Show one message at a time

// Radio Settings Variables
int radioVolume = 50;
int radioSquelch = 20;
bool radioGPSDisplay = true;
bool radioGPSA = true;
int radioGPSBaudRate = 9600;
int radioFrequency = 145000000; // Default to 2m band
String radioMode = "FM"; // Default mode
int radioPowerLevel = 50; // Power level in watts
String radioDStarCallSign = ""; // D-STAR callsign
String radioDStarMessage = ""; // D-STAR message
int radioMemoryChannel = 0; // Current memory channel
int radioSettingIndex = 0; // Current setting being edited
bool isEditingSetting = false; // Whether we're in edit mode

// Radio GPS Status Variables
String lastRadioNMEA = "";
unsigned long lastRadioNMEATime = 0;
bool radioGPSFix = false;
float radioLat = 0.0, radioLng = 0.0;
int radioSats = 0;
String radioGPSStatus = "No GPS data";

// Add these variables near the top with other global variables
static unsigned long lastButtonReleaseTime = 0;
static bool waitingForDoubleClick = false;
const unsigned long DOUBLE_CLICK_TIMEOUT = 300; // 300ms to detect double click

// Add this near the top with other global variables

// Add this near the top with other global variables
static bool ignoreNextRelease = false;

// Add this near the top with other global variables
static unsigned long lastRadioStatusQuery = 0;
static bool lastRadioStatus = false;
const unsigned long RADIO_STATUS_QUERY_INTERVAL = 5000; // 5 seconds

void setup() {
  // Initialize M5 hardware with proper configuration for M5Atom Echo
  auto cfg = M5.config();
  cfg.serial_baudrate = 0;  // Disable M5 serial
  M5.begin(cfg);

  // Configure button timing
  M5.BtnA.setDebounceThresh(20);  // 20ms debounce
  M5.BtnA.setHoldThresh(1000);    // 1000ms for long press

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialize display first
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // If display fails, we can't show anything
    logMessage("ERROR: SSD1306 display initialization failed");
    return;
  }
  display_initialized = true;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Starting...");
  display.display();

  // Initialize Radio serial for NMEA data only
  Radio.begin(9600, SERIAL_8N1, RADIO_RX, RADIO_TX);
  
  // Initialize ICOM7100 configurator
  radioConfig = new ICOM7100Configurator(Radio);
  radioConfig->initialize();
  
  logMessage("System initializing...");

  // --- Initialize GPS with Configuration --- 
  logMessage("Initializing GPS module...");
  gpsBaudRate = 9600; // Set to 9600 baud to match radio
  bool gpsInitSuccess = false;

  // Basic GPS serial start needed before configuration
  GPS.begin(gpsBaudRate, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(100); // Small delay for serial port

  if (GPS) { // Check if serial port opened successfully
    logMessage("GPS Serial Port OK. Configuring NMEA...");
    GPSConfigurator gpsConfig(GPS);

    // Set baud rate to match gpsBaudRate
    if (!gpsConfig.setBaudRate(gpsBaudRate)) {
      logMessage("Failed to set GPS baud rate (continuing)");
    }
    delay(GPS_CONFIG_RETRY_DELAY); // Delay between commands

    // Set update rate to 1Hz (standard for most GPS receivers)
    if (!gpsConfig.setUpdateRateHz(2)) {
      logMessage("Failed to set GPS update rate (continuing)");
    }
    delay(GPS_CONFIG_RETRY_DELAY); // Delay between commands

    // Set dynamic model to Portable (0)
    if (!gpsConfig.setDynamicModel(0)) {
      logMessage("Failed to set GPS dynamic model (continuing)");
    }
    delay(GPS_CONFIG_RETRY_DELAY); // Delay between commands

      // Enable essential NMEA messages (GGA, RMC, VTG, GSA, GSV, GLL)
    logMessage("Attempting to enable NMEA Messages...");
    const uint8_t nmeaClass = 0xF0;
   // const uint8_t msgIds[] = {0x00, 0x04, 0x05, 0x02, 0x03, 0x01};
   // const char* msgNames[] = {"GGA", "RMC", "VTG", "GSA", "GSV", "GLL"};
       const uint8_t msgIds[] = {0x00, 0x04, 0x05, 0x02, 0x03, 0x01};
    const char* msgNames[] = {"GGA", "RMC", "VTG", "GSA", "GSV", "GLL"};
    bool anyNmeaFail = false;

    for (size_t i = 0; i < sizeof(msgIds) / sizeof(msgIds[0]); ++i) {
        logMessage("  Sending enable command for: " + String(msgNames[i]) + "...");
        // enableNmeaMessage sends the command but doesn't wait for ACK/NACK
        if (gpsConfig.enableNmeaMessage(nmeaClass, msgIds[i], true)) {
             logMessage("    -> Command sent successfully."); 
        } else {
            logMessage("ERROR: Failed to *send* enable command for " + String(msgNames[i]));
            anyNmeaFail = true; 
        }
        delay(GPS_CONFIG_RETRY_DELAY / 2); 
    }

    if (anyNmeaFail) {
        logMessage("Warning: Failed to *send* one or more NMEA enable commands.");
    } else {
        logMessage("All NMEA enable commands sent successfully.");
    }

    logMessage("Attempting to save GPS configuration...");
    if (gpsConfig.saveConfiguration()) {
      logMessage("GPS configuration save command sent.");
      gpsInitSuccess = true;
    } else {
      logMessage("Failed to send save GPS configuration command.");
    }
  } else {
    logMessage("*** ERROR: Failed to open GPS Serial Port! ***");
    gpsInitSuccess = false;
  }

  gpsInitialized = gpsInitSuccess;
  if (gpsInitialized) {
      logMessage("GPS initialization sequence complete (check logs for details).");
  } else {
      logMessage("*** WARNING: GPS initialization sequence failed or incomplete. ***");
  }
  // --- End GPS Initialization ---

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
 
  
  // Load other settings (declination, inversion, etc.)
  compassInverted = preferences.getBool(KEY_COMPASS_INVERTED, false);
  altitudeCorrection = preferences.getInt(KEY_ALT_CORRECTION, 0);
  if (activeCompass == &hmcCompass) {
      currentDeclination = preferences.getFloat(KEY_DECLINATION, 0.0f);
      hmcCompass.setDeclination(currentDeclination);
  }

  preferences.end();

  // Set initial display mode
  currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
  
  // Log the active settings
  logMessage("--- Active Settings ---");
  logMessage("M5 Smoothing: " + String(useM5StackSmoothing ? "ON" : "OFF"));
  logMessage("M5 Interference: " + String(useM5StackInterference ? "ON" : "OFF"));

  logMessage("-----------------------");

  // Show ready message
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("System Ready");
  display.display();
  delay(1000);  // Show ready message for 1 second

  // Initialize the button handler map
  buttonHandlerMap[UIState::GPS_STATUS_SCREEN] = ButtonHandlers{handleShortPressGPSStatus, handleLongPressGPSStatus, nullptr};
  buttonHandlerMap[UIState::COMPASS_STATUS_SCREEN] = ButtonHandlers{handleShortPressCompassStatus, handleLongPressCompassStatus, handleDoubleClickCompassStatus};
  buttonHandlerMap[UIState::GRAPHIC_COMPASS_SCREEN] = ButtonHandlers{handleShortPressGraphicCompass, handleLongPressGraphicCompass, nullptr};
  buttonHandlerMap[UIState::WORLD_MAP_SCREEN] = ButtonHandlers{handleShortPressWorldMap, handleLongPressWorldMap, nullptr};
  buttonHandlerMap[UIState::ALTITUDE_CORRECTION_MODE] = ButtonHandlers{handleShortPressAltitudeCorrection, handleLongPressAltitudeCorrection, nullptr};
  buttonHandlerMap[UIState::CALIBRATION_MODE_SELECTION] = ButtonHandlers{handleShortPressCalibrationModeSelection, handleLongPressCalibrationModeSelection, nullptr};
  buttonHandlerMap[UIState::CALIBRATING_COMPASS] = ButtonHandlers{handleShortPressCalibrating, handleLongPressCalibrating, nullptr};
  buttonHandlerMap[UIState::SETTING_DECLINATION] = ButtonHandlers{handleShortPressDeclination, handleLongPressDeclination, nullptr};
  buttonHandlerMap[UIState::SETTING_INVERSION] = ButtonHandlers{handleShortPressInversion, handleLongPressInversion, nullptr};
  buttonHandlerMap[UIState::LOG_DISPLAY] = ButtonHandlers{handleShortPressLogDisplay, handleLongPressLogDisplay, nullptr};
  buttonHandlerMap[UIState::RADIO_SETTINGS_SCREEN] = ButtonHandlers{handleShortPressRadioSettings, handleLongPressRadioSettings, handleDoubleClickRadioSettings};
  buttonHandlerMap[UIState::RADIO_STATUS_SCREEN] = ButtonHandlers{handleShortPressRadioStatus, handleLongPressRadioStatus, nullptr};
  
  
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

  // After all initialization is complete (at the end of setup()):
  logMessage("Setup complete.");
  logMessage("Ready for operation.");
}

void loop() {
  // Basic hardware update - must happen first
  M5.update();

  // Determine current UI state before handling button presses
  currentUIState = determineCurrentUIState();

  // Handle button presses based on current UI state
  if (M5.BtnA.wasHold()) {
    if (buttonHandlerMap[currentUIState].longPressHandler) {
      buttonHandlerMap[currentUIState].longPressHandler();
      ignoreNextRelease = true; // Set flag to ignore next release
    }
    waitingForDoubleClick = false; // Reset double click state
  }
  else if (M5.BtnA.wasReleased()) {
    if (ignoreNextRelease) {
      // Ignore this release and reset the flag
      ignoreNextRelease = false;
      waitingForDoubleClick = false;
      return;
    }

    unsigned long currentTime = millis();
    
    if (waitingForDoubleClick && (currentTime - lastButtonReleaseTime < DOUBLE_CLICK_TIMEOUT)) {
      // Double click detected
      if (buttonHandlerMap[currentUIState].doubleClickHandler) {
        buttonHandlerMap[currentUIState].doubleClickHandler();
      }
      waitingForDoubleClick = false;
    } else {
      // First click detected, wait for potential second click
      waitingForDoubleClick = true;
      lastButtonReleaseTime = currentTime;
    }
  }
  else if (waitingForDoubleClick && (millis() - lastButtonReleaseTime >= DOUBLE_CLICK_TIMEOUT)) {
    // Timeout reached, treat as single click
    if (buttonHandlerMap[currentUIState].shortPressHandler) {
      buttonHandlerMap[currentUIState].shortPressHandler();
    }
    waitingForDoubleClick = false;
  }

  // Add static variable to track last compass update time
  static unsigned long lastCompassUpdate = 0;
  const unsigned long COMPASS_UPDATE_INTERVAL = 100; // Update compass every 100ms (10Hz)

  // Read compass data if available and enough time has passed
  if (activeCompass != nullptr && (millis() - lastCompassUpdate >= COMPASS_UPDATE_INTERVAL)) {
    activeCompass->read();
    lastCompassUpdate = millis();
  }

  // Process any available GPS data
  if (gpsInitialized) {
    while (GPS.available()) {
      gps.encode(GPS.read());
    }
    
    // Forward NMEA messages to Radio
    radioConfig->forwardNMEAToRadio(gps, altitudeCorrection);
  }

  // Read and parse NMEA from Radio
  while (Radio.available()) {
    String nmea = Radio.readStringUntil('\n');
    nmea.trim();
    if (nmea.startsWith("$GPGGA")) {
      lastRadioNMEA = nmea;
      lastRadioNMEATime = millis();
      
      // Parse GGA for fix, lat, lng, sats
      int idx = 0;
      String parts[15];
      int lastIdx = 0;
      for (int i = 0; i < nmea.length() && idx < 15; i++) {
        if (nmea[i] == ',' || nmea[i] == '*') {
          parts[idx++] = nmea.substring(lastIdx, i);
          lastIdx = i + 1;
        }
      }
      
      if (idx >= 7) {
        radioGPSFix = (parts[6].toInt() > 0);
        if (radioGPSFix) {
          radioLat = parts[2].toFloat();
          radioLng = parts[4].toFloat();
          radioSats = parts[7].toInt();
          radioGPSStatus = "Valid GPS data";
        } else {
          radioGPSStatus = "No GPS fix";
        }
      }
    }
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
      case DisplayMode::LOG_DISPLAY:
        displayLogMessages();
        break;
      case DisplayMode::RADIO_SETTINGS:
        displayRadioSettings();
        break;
      case DisplayMode::RADIO_STATUS:
        displayRadioStatus();
        break;
    }
  }
}

// World Map Screen
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
 
 
void handleShortPressWorldMap() {
  currentDisplayMode = DisplayMode::GPS_STATUS;
  logMessage("Display mode changed to: GPS_STATUS");
}

void handleLongPressWorldMap() {
  // Toggle privacy mode
  privacyModeEnabled = !privacyModeEnabled;
  logMessage("Privacy mode " + String(privacyModeEnabled ? "enabled" : "disabled"));
  
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
            
            logMessage("HMC5883L calibration and interference settings saved.");
          }
          preferences.end();
          
          // Exit calibration mode
          isCalibrating = false;
          calibrationStep = 0;
          
          // Move to inversion setting
          isSettingInversion = true;
        
          logMessage("Applied interference settings: ");
 
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

void displayCompassLogOnOLED() {
  if (!display_initialized) return;

  // Add static variable to track last update time
  static unsigned long lastUpdateTime = 0;
  const unsigned long UPDATE_INTERVAL = 100; // Update every 100ms (10Hz)

  // Check if enough time has passed since last update
  if (millis() - lastUpdateTime < UPDATE_INTERVAL) {
    return; // Skip this update
  }
  lastUpdateTime = millis();

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
void handleShortPressCompassStatus() {
  // Go to radio status screen
  currentDisplayMode = DisplayMode::RADIO_STATUS;
  logMessage("Display mode changed to: RADIO_STATUS");
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

void displayLogOnOLED() {
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
  isSettingAltitudeCorrection = true;
  logMessage("Entering altitude correction mode...");
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
void handleShortPressGraphicCompass() {
 
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

void displayLogMessages() {
  if (!display_initialized) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Center the title
  String title = "--- Error Log ---";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 0);
  display.println(title);

  // Handle empty log buffer
  if (log_buffer.empty()) {
    display.setCursor(0, 20);
    display.println("No Errors");
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print("Click: Next Screen");
    display.display();
    return;
  }

  // Ensure current index is valid
  if (currentLogIndex >= log_buffer.size()) {
    currentLogIndex = log_buffer.size() - 1;
  }
  if (currentLogIndex < 0) {
    currentLogIndex = 0;
  }

  // Display the current message
  if (!log_buffer.empty()) {
    // Calculate which message to show (most recent first)
    int displayIndex = log_buffer.size() - 1 - currentLogIndex;
    
    // Display the message with word wrapping
    String message = log_buffer[displayIndex];
    int yPos = 12; // Start below the title
    int maxWidth = SCREEN_WIDTH - 2; // Leave small margin
    
    // Set text color to inverse for all messages (since they're all errors/warnings)
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    
    // Split message into words and wrap
    int currentX = 0;
    String currentLine = "";
    
    for (int i = 0; i < message.length(); i++) {
      char c = message[i];
      currentLine += c;
      display.getTextBounds(currentLine, 0, 0, &x1, &y1, &w, &h);
      
      if (w > maxWidth || c == '\n') {
        // Print the line (excluding the last character that caused overflow)
        if (c == '\n') {
          display.setCursor(0, yPos);
          display.println(currentLine.substring(0, currentLine.length() - 1));
        } else {
          display.setCursor(0, yPos);
          display.println(currentLine.substring(0, currentLine.length() - 1));
          i--; // Back up one character to process it in the next line
        }
        yPos += 8; // Move to next line
        currentLine = "";
      }
    }
    
    // Print any remaining text
    if (currentLine.length() > 0) {
      display.setCursor(0, yPos);
      display.println(currentLine);
    }
  }

  // Show navigation info at bottom
  display.setTextColor(SSD1306_WHITE); // Reset to normal text color
  display.setCursor(0, SCREEN_HEIGHT - 16);
  display.print("Error ");
  display.print(currentLogIndex + 1);
  display.print("/");
  display.print(log_buffer.size());
  
  display.setCursor(0, SCREEN_HEIGHT - 8);
  display.print("Click: Cycle  Hold: Next Screen");

  display.display();
}

void handleShortPressLogDisplay() {
  // If no errors, go to next screen immediately
  if (log_buffer.empty()) {
    currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
    logMessage("Display mode changed to: GRAPHIC_COMPASS");
    return;
  }
  
  // Navigate to next older message
  if (!log_buffer.empty()) {
    currentLogIndex = (currentLogIndex + 1) % log_buffer.size();
    logMessage("Viewing error " + String(currentLogIndex + 1) + " of " + String(log_buffer.size()));
  }
}

void handleLongPressLogDisplay() {


}

// Add this function to handle entering the log display mode
void enterLogDisplayMode() {
  currentLogIndex = 0; // Reset to most recent message
  currentDisplayMode = DisplayMode::LOG_DISPLAY;
  displayLogMessages();
}


// Add new display function
void displayRadioSettings() {
  if (!display_initialized) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Center the title
  String title = "--- Radio Settings ---";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 0);
  display.println(title);

  // Define the settings area (leave space for command instructions at bottom)
  const int SETTINGS_START_Y = 12;
  const int SETTINGS_END_Y = SCREEN_HEIGHT - 24; // Leave space for scroll indicator and commands
  const int SETTINGS_HEIGHT = SETTINGS_END_Y - SETTINGS_START_Y;
  const int ITEMS_PER_PAGE = 4; // Number of settings to show at once

  // Calculate which settings to show based on current index
  int startIndex = (radioSettingIndex / ITEMS_PER_PAGE) * ITEMS_PER_PAGE;
  int endIndex = min(startIndex + ITEMS_PER_PAGE, 7); // 7 total items (5 settings + 2 actions)

  // Display current settings with highlight for selected setting
  for (int i = startIndex; i < endIndex; i++) {
    int displayY = SETTINGS_START_Y + ((i - startIndex) * 10);
    display.setCursor(0, displayY);
    
    if (i == radioSettingIndex) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    
    switch (i) {
      case 0:
        display.print("Freq: ");
        display.print(radioFrequency / 1000000.0, 3);
        display.print("MHz");
        break;
      case 1:
        display.print("Mode: ");
        display.print(radioMode);
        break;
      case 2:
        display.print("Power: ");
        display.print(radioPowerLevel);
        display.print("W");
        break;
      case 3:
        display.print("Mem: ");
        display.print(radioMemoryChannel);
        break;
      case 4:
        display.print("D-STAR: ");
        if (radioDStarCallSign.length() > 0) {
          display.print(radioDStarCallSign);
        } else {
          display.print("OFF");
        }
        break;
      case 5:
        display.print("Save & Exit");
        break;
      case 6:
        display.print("Exit");
        break;
    }
  }

  // Show scroll indicator if there are more settings
  if (startIndex > 0 || endIndex < 7) {
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(SCREEN_WIDTH - 8, SETTINGS_START_Y);
    if (startIndex > 0) {
      display.print("^"); // Up arrow
    }
    display.setCursor(SCREEN_WIDTH - 8, SETTINGS_END_Y - 8);
    if (endIndex < 7) {
      display.print("v"); // Down arrow
    }
  }

  // Show navigation info at bottom
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, SCREEN_HEIGHT - 16);
  if (isEditingSetting) {
    display.print("Clk: ^ Dbl: v Hold:");
    // Draw save icon
    int x = display.getCursorX();
    int y = display.getCursorY();
    // Disk body
    display.fillRect(x, y, 8, 8, SSD1306_WHITE);
    // Disk label
    display.drawRect(x + 2, y + 2, 4, 4, SSD1306_BLACK);
    // Disk shutter
    display.drawRect(x - 1, y + 1, 2, 6, SSD1306_WHITE);
  } else {
    display.print("Clk:Next  Hold:Edit");
  }

  display.display();
}

void handleShortPressRadioSettings() {
  if (isEditingSetting) {
    // Increment current setting
    switch (radioSettingIndex) {
      case 0: // Frequency
        radioFrequency += 1000000; // Increment by 1MHz
        if (radioFrequency > 148000000) radioFrequency = 144000000;
        break;
      case 1: // Mode
        if (radioMode == "FM") radioMode = "AM";
        else if (radioMode == "AM") radioMode = "LSB";
        else if (radioMode == "LSB") radioMode = "USB";
        else if (radioMode == "USB") radioMode = "CW";
        else if (radioMode == "CW") radioMode = "FM";
        break;
      case 2: // Power Level
        radioPowerLevel += 5;
        if (radioPowerLevel > 100) radioPowerLevel = 5;
        break;
      case 3: // Memory Channel
        radioMemoryChannel = (radioMemoryChannel + 1) % 100;
        break;
      case 4: // D-STAR
        if (radioDStarCallSign.length() == 0) {
          radioDStarCallSign = "N0CALL";
          radioDStarMessage = "Hello";
        } else {
          radioDStarCallSign = "";
          radioDStarMessage = "";
        }
        break;
    }
  } else {
    // Select next setting/action
    radioSettingIndex = (radioSettingIndex + 1) % 7;
  }
  displayRadioSettings();
}

void handleDoubleClickRadioSettings() {
  if (isEditingSetting) {
    // Decrement current setting
    switch (radioSettingIndex) {
      case 0: // Frequency
        radioFrequency -= 1000000; // Decrement by 1MHz
        if (radioFrequency < 144000000) radioFrequency = 148000000;
        break;
      case 1: // Mode
        if (radioMode == "FM") radioMode = "CW";
        else if (radioMode == "CW") radioMode = "USB";
        else if (radioMode == "USB") radioMode = "LSB";
        else if (radioMode == "LSB") radioMode = "AM";
        else if (radioMode == "AM") radioMode = "FM";
        break;
      case 2: // Power Level
        radioPowerLevel -= 5;
        if (radioPowerLevel < 5) radioPowerLevel = 100;
        break;
      case 3: // Memory Channel
        radioMemoryChannel = (radioMemoryChannel - 1 + 100) % 100;
        break;
      case 4: // D-STAR
        if (radioDStarCallSign.length() == 0) {
          radioDStarCallSign = "N0CALL";
          radioDStarMessage = "Hello";
        } else {
          radioDStarCallSign = "";
          radioDStarMessage = "";
        }
        break;
    }
    displayRadioSettings();
  }
  // No action in navigation mode
}

void handleLongPressRadioSettings() {
  if (isEditingSetting) {
    // Set current value and exit edit mode
    isEditingSetting = false;
    logMessage("Setting saved");
  } else {
    // Handle actions or enter edit mode
    if (radioSettingIndex == 5) { // Save & Exit
      currentDisplayMode = DisplayMode::LOG_DISPLAY;
      logMessage("Radio settings saved");
      return;
    } else if (radioSettingIndex == 6) { // Exit
      currentDisplayMode = DisplayMode::LOG_DISPLAY;
      logMessage("Radio settings not saved");
      return;
    } else {
      // Enter edit mode for current setting
      isEditingSetting = true;
      logMessage("Editing " + String(radioSettingIndex == 0 ? "Frequency" : 
                                   radioSettingIndex == 1 ? "Mode" :
                                   radioSettingIndex == 2 ? "Power" :
                                   radioSettingIndex == 3 ? "Memory" : "D-STAR"));
    }
  }
  displayRadioSettings();
}

void displayRadioStatus() {
    if (!display_initialized) return;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Center the title
    String title = "--- Radio GPS Status ---";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println(title);

    // Display radio CI-V status
    display.setCursor(0, 12);
    display.print("CI-V: ");
    
    // Only query radio status every 5 seconds
    unsigned long currentTime = millis();
    if (currentTime - lastRadioStatusQuery >= RADIO_STATUS_QUERY_INTERVAL) {
        lastRadioStatus = radioConfig->queryStatus();
        lastRadioStatusQuery = currentTime;
    }
    display.println(lastRadioStatus ? "OK" : "No Response");

    // Display radio connection status
    display.setCursor(0, 22);
    display.print("NMEA: ");
    // Check if we've received any data in the last 5 seconds
    bool radioConnected = (millis() - lastRadioNMEATime < 5000);
    if (radioConnected) {
        display.println("Connected");
    } else {
        display.println("Not Connected");
        // If not connected, show the time since last data
        display.setCursor(0, 32);
        display.print("Last data: ");
        if (lastRadioNMEATime > 0) {
            display.print((millis() - lastRadioNMEATime) / 1000);
            display.println("s ago");
        } else {
            display.println("Never");
        }
        display.display();
        return; // Don't show GPS data if radio isn't connected
    }

    // Display GPS status
    display.setCursor(0, 32);
    display.print("Status: ");
    display.println(radioGPSStatus);

    display.setCursor(0, 42);
    display.print("Fix: ");
    display.println(radioGPSFix ? "YES" : "NO");

    if (radioGPSFix) {
        display.setCursor(0, 52);
        display.print("Lat: ");
        display.println(radioLat, 5);
        
        display.setCursor(0, 62);
        display.print("Lng: ");
        display.println(radioLng, 5);
    } else {
        // Show last NMEA message if no fix
        display.setCursor(0, 52);
        if (lastRadioNMEA.length() > 0) {
            display.print("Last: ");
            String truncated = lastRadioNMEA.substring(0, 20);
            display.println(truncated);
        } else {
            display.println("No NMEA data");
        }
    }

    // Navigation instructions
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print("Click:Log  Hold:Settings");

    display.display();
}

void handleShortPressRadioStatus() {
    // Go to log display screen
    currentDisplayMode = DisplayMode::LOG_DISPLAY;
    logMessage("Display mode changed to: LOG_DISPLAY");
}

void handleLongPressRadioStatus() {
    // Switch to next screen
  currentDisplayMode = DisplayMode::RADIO_SETTINGS;
  logMessage("Display mode changed to: RADIO_SETTINGS");
}

void resetCompass() {
  // Clear all compass preferences
  preferences.begin(PREF_NAMESPACE, false);
  preferences.clear();
  preferences.end();
  
  // Reset compass state variables
  compassInverted = false;
  currentDeclination = 0.0f;
  isCalibrating = false;
  isSettingDeclination = false;
  isSelectingCalibrationMode = false;
  isSettingInversion = false;
  calibrationStep = 0;
  
  // Reinitialize compass
  if (qmcCompass.begin()) {
    activeCompass = &qmcCompass;
    logMessage("QMC5883L reinitialized after reset");
  } else if (hmcCompass.begin()) {
    activeCompass = &hmcCompass;
    logMessage("HMC5883L reinitialized after reset");
  } else {
    activeCompass = nullptr;
    logMessage("Compass reinitialization failed after reset");
  }
  
  // If we have a working compass, read initial heading
  if (activeCompass) {
    activeCompass->read();
    int initialHeading = activeCompass->getAzimuth();
    char dirStr[4];
    activeCompass->getDirection(dirStr, initialHeading);
    logMessage("Initial compass heading after reset: " + String(initialHeading) + "° (" + String(dirStr) + ")");
  }
}

void handleDoubleClickCompassStatus() {
  // Reset compass on double click
  resetCompass();
  logMessage("Compass reset triggered by double click");
}
