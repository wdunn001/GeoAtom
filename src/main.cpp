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
#include <deque>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_wifi.h>
#include <DNSServer.h>
#include "radio_web_server.h"
#include "display_world_map.h"
#include "display_compass_status.h"
#include "display_manager.h"
#include "display_graphic_compass.h"
#include "display_gps_status.h"
#include "display_log.h"
#include "display_radio_status.h"
#include "display_radio_settings.h"
#include "display_wifi_status.h"
#include "main_globals.h"
#include "yuma_http_service.h"
#include "ble_service.h"
#include "display_ble_status.h"
extern bool wifiSetupEnabled;
extern void displayWiFiStatus();
extern void setupWiFiAndWeb();

// Remove #define SCREEN_WIDTH and #define SCREEN_HEIGHT
// Instead, define them as variables for external linkage
int SCREEN_WIDTH = 128;
int SCREEN_HEIGHT = 64;

// Hand-drawn world map bitmap 128x64 pixels
const unsigned char world_map[] PROGMEM = {
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
void displayWorldMap();
void displayRadioSettings();
void displayRadioStatus();
void calculateAndApplyCalibration(); // New calibration function
void setDeclinationFromGPS(float lat, float lng); // Auto declination based on GPS position
void applyPrivacyFilter(); // Stub - Replaced by getLatitude() and getLongitude() wrappers
void displayWiFiStatus();
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
UsbRadio usbRadio(Radio);
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
bool wifiSetupEnabled = false;
// Calibration mode options
const int NUM_CALIBRATION_MODES = 4;
const char* CALIBRATION_MODE_NAMES[NUM_CALIBRATION_MODES] = {
  "4-Point Cal", "8-Point Cal", "16-Point Cal", "Cancel"
};
const int CALIBRATION_MODE_POINTS[NUM_CALIBRATION_MODES] = {
  4, 8, 16, 0
};

// SSD1306 display configuration
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
// (REMOVED: enum class DisplayMode ...)
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
const char* KEY_RADIO_USB_MODE = "radioUsbMode";     // Key for Radio USB mode

// Pin definitions based on physical connections
// --- Verified Working Configuration for BN-880 & M5Atom Echo ---
// I2C pins - shared between compass (BN-880) and display (SSD1306)
#define I2C_SDA 25 // Grove Pin 1
#define I2C_SCL 21 // Grove Pin 2

// GPS module connections (BN-880 UART)
#define GPS_RX 23 // ESP32 RX pin connected to BN-880 TX
#define GPS_TX 33  // ESP32 TX pin connected to BN-880 RX

// TTL to RS232 level converter connections (Optional Radio Output)
#define RADIO_RX 19  // ESP32 RX pin connected to TTL TX
#define RADIO_TX 22  // ESP32 TX pin connected to TTL RX
// --------------------------------------------------------------

// Create a global log instance
m5::Log_Class m5Log;

// Create ICOM7100 configurator instance
ICOM7100Configurator* radioConfig = nullptr;

// Batched logging
static std::deque<String> log_batch;
static String lastMagnetoMsg = "";
static unsigned long lastLogFlush = 0;
const unsigned long LOG_FLUSH_INTERVAL = 2000; // ms

void logMessage(const String& msg) {
    // Only keep the last magnetometer message in the batch
    if (msg.startsWith("Using magnetometer for heading")) {
        lastMagnetoMsg = msg;
    } else {
        log_batch.push_back(msg);
    }
}

void flushLogMessages() {
    // Flush all batched messages
    while (!log_batch.empty()) {
        m5Log.println(log_batch.front().c_str());
        log_batch.pop_front();
    }
    // Flush the last magnetometer message (if any)
    if (!lastMagnetoMsg.isEmpty()) {
        m5Log.println(lastMagnetoMsg.c_str());
        lastMagnetoMsg = "";
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
// (REMOVED: enum class UIState ...)

// Current UI state
// UIState currentUIState = UIState::GRAPHIC_COMPASS_SCREEN;

// Function to get the current UI state based on display mode and flags
// UIState determineCurrentUIState() {
//   ...
// }

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
void handleShortPressWiFiStatus();
void handleLongPressWiFiStatus();
void handleDoubleClickRadioSettings();
void handleDoubleClickCompassStatus();
void handleShortPressBLEStatus();
void handleLongPressBLEStatus();
// Function prototypes
void scanI2CDevices();
void forwardNMEAToICOM(TinyGPSPlus& gps, int altitudeCorrection);

// --- Restored global variables for modularized code ---
unsigned long GPS_CONFIG_RETRY_DELAY = 100; // 100ms between retries
bool gpsInitialized = false;
bool radioUsbMode = false;
bool waitingForDoubleClick = false;
unsigned long lastButtonReleaseTime = 0;
const unsigned long DOUBLE_CLICK_TIMEOUT = 300; // 300ms to detect double click
String lastRadioNMEA = "";
unsigned long lastRadioNMEATime = 0;
bool radioGPSFix = false;
float radioLat = 0.0, radioLng = 0.0;
int radioSats = 0;
String radioGPSStatus = "No GPS data";
float speedHistory[20] = {0}; // 20 = SPEED_AVG_SAMPLES
int speedHistoryIndex = 0;
const int SPEED_AVG_SAMPLES = 20;
unsigned long lastCheckTime = 0;
float lastCheckLat = 0.0f, lastCheckLng = 0.0f;
const float MOVEMENT_DIST_THRESHOLD = 5.0f;
const float SPEED_THRESHOLD = 2.0f;
int movingCounter = 0;
int notMovingCounter = 0;
bool filteredIsMoving = false;
const int MOVING_DEBOUNCE = 3;
bool isActuallyMoving = false;

// --- MISSING GLOBALS AND FUNCTIONS FOR LINKER ---

// Add missing global variables
int currentLogIndex = 0;
int radioSettingIndex = 0;
bool isEditingSetting = false;
int radioFrequency = 145000000;
String radioMode = "FM";
int radioPowerLevel = 50;
int radioMemoryChannel = 0;
String radioDStarCallSign = "";
String radioDStarMessage = "";

// Add missing function implementations
float getSmartSpeed() {
    // TODO: Replace with real logic
    return gps.speed.isValid() ? gps.speed.kmph() : 0.0f;
}

float getSmartHeading() {
    // TODO: Replace with real logic
    if (activeCompass) {
        int heading = activeCompass->getAzimuth();
        if (heading >= 0) return (float)heading;
    }
    return gps.course.isValid() ? gps.course.deg() : 0.0f;
}

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
    if (!gpsConfig.setUpdateRateHz(1)) {
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
  
  // Load radio settings
  radioUsbMode = preferences.getBool(KEY_RADIO_USB_MODE, false);
 
  
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
  setupButtonHandlers();

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

  setupWiFiAndWeb();
  // Set wifiSetupEnabled based on current WiFi mode after WiFi is initialized
  if (WiFi.getMode() & WIFI_AP) {
    wifiSetupEnabled = true;
  } else {
    wifiSetupEnabled = false;
  }

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

  // Initialize BLE and Yuma services
  initBLEService();
  initYumaHttpService();

  // Ensure Yuma Almanac is up-to-date if we have internet access
  if (WiFi.status() == WL_CONNECTED) {
    if (ensureYumaAlmanacCurrent()) {
      logMessage("Yuma almanac is up-to-date.");
    } else {
      logMessage("Failed to update Yuma almanac (no internet or error).");
    }
  } else {
    logMessage("WiFi not connected, skipping Yuma almanac update.");
  }
}

void loop() {
  // Basic hardware update - must happen first
  M5.update();

  // Determine current UI state before handling button presses
  extern UIState determineCurrentUIState();
  extern std::map<UIState, ButtonHandlers> buttonHandlerMap;
  static bool ignoreNextRelease = false;
  UIState currentUIState = determineCurrentUIState();

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
    
    // Forward NMEA messages based on configuration
    if (radioConfig != nullptr) {
      // Always forward to radio
      radioConfig->forwardNMEAToRadio(gps, altitudeCorrection);
    }
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
        displayGPSStatusOnOLED();
        break;
      case DisplayMode::COMPASS_STATUS:
        displayCompassStatusOnOLED();
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
      case DisplayMode::WIFI_STATUS:
        displayWiFiStatus();
        break;
      case DisplayMode::BLE_STATUS:
        displayBLEStatus();
        break;
    }
  }

  // Update speed history every 500ms
  static unsigned long lastSpeedUpdate = 0;
  if (millis() - lastSpeedUpdate > 500) {
    lastSpeedUpdate = millis();
    float spd = gps.speed.isValid() ? gps.speed.kmph() : 0.0f;
    speedHistory[speedHistoryIndex] = spd;
    speedHistoryIndex = (speedHistoryIndex + 1) % SPEED_AVG_SAMPLES;
  }
  // Calculate average speed
  float avgSpeed = 0.0f;
  for (int i = 0; i < SPEED_AVG_SAMPLES; ++i) avgSpeed += speedHistory[i];
  avgSpeed /= SPEED_AVG_SAMPLES;
  // Improved movement detection with debounce and higher threshold
  if (millis() - lastCheckTime > 2000) {
    lastCheckTime = millis();
    if (gps.location.isValid()) {
      float curLat = gps.location.lat();
      float curLng = gps.location.lng();
      float dLat = curLat - lastCheckLat;
      float dLng = curLng - lastCheckLng;
      float dist = sqrt(dLat * dLat + dLng * dLng) * 111320.0f; // meters per degree
      bool currentlyMoving = (dist > MOVEMENT_DIST_THRESHOLD) && (avgSpeed > SPEED_THRESHOLD);
      if (currentlyMoving) {
        movingCounter++;
        notMovingCounter = 0;
      } else {
        notMovingCounter++;
        movingCounter = 0;
      }
      if (movingCounter >= MOVING_DEBOUNCE) filteredIsMoving = true;
      if (notMovingCounter >= MOVING_DEBOUNCE) filteredIsMoving = false;
      isActuallyMoving = filteredIsMoving;
      lastCheckLat = curLat;
      lastCheckLng = curLng;
    }
  }

  // Flush batched log messages every LOG_FLUSH_INTERVAL
  if (millis() - lastLogFlush > LOG_FLUSH_INTERVAL) {
      flushLogMessages();
      lastLogFlush = millis();
  }
  
  // Process DNS requests for captive portal
  if (WiFi.getMode() & WIFI_AP) {
    getDnsServer().processNextRequest();
  }

  // Only bridge USB <-> radio if radioUsbMode is enabled
  if (radioUsbMode) {
    usbRadio.loop();
  }
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







