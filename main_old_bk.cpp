#include <Arduino.h>
#include <HardwareSerial.h>
#include <M5Unified.h>
#include <TinyGPS++.h>
#include "CompassInterface.h"
#include <vector>
#include <String>
#include <Preferences.h>
#include "GPSConfigurator.h"

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
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
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

// GPS setup
TinyGPSPlus gps;
HardwareSerial GPS(1);  // Using UART1 for GPS
HardwareSerial Host(2); // Using UART2 for NMEA forwarding to PC/TTL converter
unsigned long gpsBaudRate = 0; // Variable to store GPS baud rate
int altitudeCorrection = 1000; // Altitude correction factor in meters

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
int calibrationModeIndex = 0; // Index for current selection in calibration mode menu

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

void setup() {
  delay(1000);  // Give peripherals time to initialize
  
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

  // 1. Set Update Rate to 10Hz
  if (gpsConfig.setUpdateRateHz(10)) {
      logMessage("GPS: Set update rate to 10 Hz.");
  } else {
      logMessage("GPS: Failed to set update rate to 10 Hz.");
  }
  delay(100); // Small delay between commands

  // 2. Set Dynamic Model to Portable (0)
  if (gpsConfig.setDynamicModel(0)) {
      logMessage("GPS: Set dynamic model to Portable.");
  } else {
      logMessage("GPS: Failed to set dynamic model.");
  }
  delay(100); // Small delay between commands

  // 3. Save configuration to make it persistent
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
          
          // Load HMC declination from preferences
          preferences.begin(PREF_NAMESPACE, true); // Open NVM in read-only mode
          currentDeclination = preferences.getFloat(KEY_DECLINATION, 0.0f);
          compassInverted = preferences.getBool(KEY_COMPASS_INVERTED, false);
          preferences.end();
          
          // Set declination from saved value
          hmcCompass.setDeclination(currentDeclination);
          
          logMessage("Using HMC5883L with declination: " + String(currentDeclination * 180.0 / PI, 2) + " degrees.");
      }
      else {
          logMessage("*** WARNING: Both compass initialization attempts failed. ***");
          activeCompass = nullptr;
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
}

void loop() {
  // Update M5 hardware first
  M5.update();

  // --- Button Handling Logic (Revised for Key Up/Hold) ---
  bool button_pressed = M5.BtnA.wasPressed();
  bool button_released = M5.BtnA.wasReleased();
  bool button_held = M5.BtnA.wasHold();

  // Actions differ based on current mode and calibration state
  if (currentDisplayMode == DisplayMode::COMPASS_STATUS) {
    // Check which compass is active for available features
    bool qmc_active = (activeCompass == &qmcCompass);
    bool hmc_active = (activeCompass == &hmcCompass);
    
    if (isSelectingCalibrationMode) {
      // --- Selecting calibration mode ---
      if (button_pressed) {
        // Rotate through options
        calibrationModeIndex = (calibrationModeIndex + 1) % NUM_CALIBRATION_MODES;
        delay(50); // Debounce
      }
      else if (button_held) {
        // Select current option
        if (calibrationModeIndex == NUM_CALIBRATION_MODES - 1) {
          // Selected Cancel
          logMessage("Calibration cancelled.");
          isSelectingCalibrationMode = false;
        } else {
          // Selected a calibration mode
          calibrationPoints = CALIBRATION_MODE_POINTS[calibrationModeIndex];
          logMessage("Starting " + String(calibrationPoints) + "-point calibration...");
          isSelectingCalibrationMode = false;
          isCalibrating = true;
          calibrationStep = 1;
        }
      }
    }
    else if (isCalibrating) {
      // --- Currently Calibrating (QMC only) ---
      if (!qmc_active) {
        // Cancel calibration if not using QMC
        logMessage("Calibration only available for QMC5883L library. Cancelling.");
        isCalibrating = false;
        calibrationStep = 0;
      }
      else if (button_pressed && calibrationStep > 0 && calibrationStep <= calibrationPoints) {
        // Capture Calibration Point on Press
        logMessage("Capturing calibration point " + String(calibrationStep) + " of " + String(calibrationPoints));
        activeCompass->read();
        calX[calibrationStep - 1] = activeCompass->getX();
        calY[calibrationStep - 1] = activeCompass->getY();
        calibrationStep++;
        delay(50); // Small delay to help debounce/prevent double step

        if (calibrationStep > calibrationPoints) {
          logMessage("Calculating " + String(calibrationPoints) + "-point calibration...");
          if (qmcCompass.calculateCalibration(calX, calY)) {
            logMessage("Calibration calculated and applied successfully.");
            
            // Save to NVM
            preferences.begin(PREF_NAMESPACE, false); // Open in write mode
            
            // Get current values from the sensor to store
            float x_offset, y_offset, z_offset;
            float x_scale, y_scale, z_scale;
            
            // For now, we'll use placeholder values
            // In a real implementation, you'd have getters for these values
            x_offset = 0; y_offset = 0; z_offset = 0;
            x_scale = 1; y_scale = 1; z_scale = 1;
            
            preferences.putFloat(KEY_X_OFFSET, x_offset);
            preferences.putFloat(KEY_Y_OFFSET, y_offset);
            preferences.putFloat(KEY_Z_OFFSET, z_offset);
            preferences.putFloat(KEY_X_SCALE, x_scale);
            preferences.putFloat(KEY_Y_SCALE, y_scale);
            preferences.putFloat(KEY_Z_SCALE, z_scale);
            preferences.putBool(KEY_CAL_VALID, true);
            preferences.end();
            
            logMessage("Calibration saved to NVM.");
          } else {
            logMessage("Calibration calculation failed. Try again.");
          }
          
          // Move to inversion setting
          isCalibrating = false;
          calibrationStep = 0;
          isSettingInversion = true;
        }
      } else if (button_held) {
        // Cancel Calibration with long press
        logMessage("Calibration cancelled.");
        isCalibrating = false;
        calibrationStep = 0;
      }
      // Ignore release events during calibration steps
    } 
    else if (isSettingDeclination) {
      // --- Currently Setting Declination (HMC only) ---
      if (!hmc_active) {
        // Cancel declination setting if not using HMC
        logMessage("Declination setting only available for HMC5883L. Cancelling.");
        isSettingDeclination = false;
      }
      else if (button_pressed) {
        // Set the current direction as true north
        activeCompass->read();
        int magneticHeading = activeCompass->getAzimuth();
        // Declination is the angle between magnetic north (0) and the current direction (which user is pointing to true north)
        currentDeclination = radians(magneticHeading); // If compass reads 0-359, true north is at "magneticHeading"
        
        // Apply declination immediately
        hmcCompass.setDeclination(currentDeclination);
        
        // Save to NVM
        preferences.begin(PREF_NAMESPACE, false); // Open in write mode
        preferences.putFloat(KEY_DECLINATION, currentDeclination);
        preferences.end();
        
        float declDegrees = currentDeclination * 180.0 / PI;
        logMessage("Declination set to: " + String(declDegrees, 1) + " degrees");
        
        // Move to inversion setting
        isSettingDeclination = false;
        isSettingInversion = true;
      }
      else if (button_held) {
        // Cancel declination setting
        logMessage("Declination setting cancelled.");
        isSettingDeclination = false;
      }
    }
    else if (isSettingInversion) {
      // --- Compass Inversion Setting UI ---
      if (button_pressed) {
        // Short press sets normal (non-inverted) mode
        compassInverted = false;
        
        // Save to NVM
        preferences.begin(PREF_NAMESPACE, false); // Open in write mode
        preferences.putBool(KEY_COMPASS_INVERTED, false);
        preferences.end();
        
        logMessage("Compass display set to normal (non-inverted) mode.");
        isSettingInversion = false;
      }
      else if (button_held) {
        // Long press sets inverted mode
        compassInverted = true;
        
        // Save to NVM
        preferences.begin(PREF_NAMESPACE, false); // Open in write mode
        preferences.putBool(KEY_COMPASS_INVERTED, true);
        preferences.end();
        
        logMessage("Compass display set to inverted mode.");
        isSettingInversion = false;
      }
    }
    else if (button_held) {
      // Long press on status screen starts calibration or declination setting
      if (activeCompass == &qmcCompass) {
        // Long press shows calibration mode selection for QMC
        logMessage("Select calibration mode...");
        isSelectingCalibrationMode = true;
        calibrationModeIndex = 1; // Default to 8-point
      } 
      else if (activeCompass == &hmcCompass) {
        // Long press starts declination setting for HMC
        logMessage("Starting declination setting (HMC5883L)...");
        isSettingDeclination = true;
      }
    }
    else if (button_released) {
      // Short press/release cycles to next mode
      currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
    }
  }
  else if (currentDisplayMode == DisplayMode::GRAPHIC_COMPASS) {
    // On Graphic Compass Screen, just handle mode cycling
    if (button_released) { // Cycle modes on RELEASE
      currentDisplayMode = DisplayMode::WORLD_MAP; // Now cycles to world map instead of GPS status
    }
    // Remove long press calibration options from compass screen
  }
  else if (currentDisplayMode == DisplayMode::WORLD_MAP) {
    // On World Map Screen, just handle mode cycling
    if (button_released) { // Cycle modes on RELEASE
      currentDisplayMode = DisplayMode::GPS_STATUS;
    }
  }
  else if (currentDisplayMode == DisplayMode::GPS_STATUS) {
    // On GPS Status Screen, just handle mode cycling
    if (button_released) { // Cycle modes on RELEASE
      currentDisplayMode = DisplayMode::COMPASS_STATUS;
    }
  }
  else {
    // --- Not on Graphic Compass or GPS Status Screen ---
    if (button_released) { // Cycle modes on RELEASE
      currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
    }
  }

  // Read GPS data and forward NMEA sentences using non-blocking approach
  unsigned long startTime = millis();
  while (GPS.available() > 0 && (millis() - startTime) < 20) {
    char c = GPS.read();
    gps.encode(c);
    Host.write(c);  // Forward NMEA data to UART2 (not logged to buffer)
  }
  
  // --- Read Compass Data ---
  int currentHeading = 0;
  String rawCompassData = "Compass: Not available";
  
  if (activeCompass != nullptr) {
    activeCompass->read(); // Read new data
    currentHeading = activeCompass->getAzimuth();
    
    // Get raw data for display
    rawCompassData = String(activeCompass->getSensorName()) + ":\n";
    rawCompassData += "X=" + String(activeCompass->getX()) + "\n";
    rawCompassData += "Y=" + String(activeCompass->getY()) + "\n";
    rawCompassData += "Z=" + String(activeCompass->getZ());
    
    // Add declination info for HMC5883L
    if (activeCompass == &hmcCompass) {
      rawCompassData += "\nDecl=" + String(currentDeclination * 180.0 / PI, 1) + " deg";
    }
  }
  
  latest_compass_raw = rawCompassData;
  // --- End Read Compass Data ---

  // Update display based on GPS state OR show log buffer (approx once per second)
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 1000) {  
    lastDisplayUpdate = millis();

    // Decide which screen to show based on currentDisplayMode
    switch (currentDisplayMode) {
      case DisplayMode::GPS_STATUS:
        // Show the GPS status screen on the OLED
        displayLogOnOLED();
        break; // End GPS_STATUS mode

      case DisplayMode::COMPASS_STATUS:
        // Show the compass status screen on the OLED
        displayCompassLogOnOLED();
        break; // End COMPASS_STATUS mode

      case DisplayMode::GRAPHIC_COMPASS:
        // Show the graphic compass screen
        displayGraphicCompass();
        break; // End GRAPHIC_COMPASS mode
        
      case DisplayMode::WORLD_MAP:
        // Show the world map with current position
        displayWorldMap();
        break; // End WORLD_MAP mode
    } // end switch (currentDisplayMode)
  } // end timed display update
}

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
  display.print("Alt: "); display.println(alt);
  
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
      display.print("\xB0"); // Degree symbol
    }
  } else {
    display.setCursor(0, 40);
    display.print("No compass");
    display.setCursor(0, 48);
    display.print("Az: ---");
  }
  // --- End Compass Display ---
  
  // Display satellite count (adjust Y position)
  display.setCursor(0, 56); // Moved down slightly
  display.print("Sats: ");
  display.println(gps.satellites.value());
  
  // --- Lat/Lon Data moved to bottom of screen ---
  if (gps.location.isValid()) {
    // Format coordinates to prevent overflow
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    
    // Display single label above values
    display.setCursor(0, SCREEN_HEIGHT - 24);
    display.println("Lat/Lng:");
    
    // Display lat value below the label
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.println(String(lat, 5));
    
    // Display lng label aligned to right edge
    String lngLabel = "Lng:";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(lngLabel, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 16);
    display.println(lngLabel);
    
    // Display lng value below its label, right aligned
    String lngValue = String(lng, 5);
    display.getTextBounds(lngValue, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 8);
    display.print(lngValue);
  } else {
    display.setCursor(0, SCREEN_HEIGHT - 24);
    display.println("Lat/Lng:");
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.println("No Fix");
    
    // Right align Lng: label for No Fix case
    String lngLabel = "Lng:";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(lngLabel, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 16);
    display.println(lngLabel);
    
    // Right align No Fix message
    String noFix = "No Fix";
    display.getTextBounds(noFix, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 8);
    display.println(noFix);
  }
  
  // Display inversion status indicator if inverted
  if (compassInverted) {
    display.setCursor(90, SCREEN_HEIGHT - 8);
    display.print("[Inv]");
  }

  display.display();
}

// Function to display the GPS STATUS on the OLED
void displayLogOnOLED() {
  if (!display_initialized) return;

  display.clearDisplay(); // Clear the entire screen first
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Center the title
  String title = "--- GPS Status ---";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 0);
  display.println(title);

  // Display GPS data format with dynamic sentence types
  String gpsProtocol = "";
  if (gps.charsProcessed() > 0) {
    // Check which NMEA sentence types have been received
    if (gps.location.isUpdated()) gpsProtocol += "GGA ";
    if (gps.date.isUpdated() || gps.time.isUpdated()) gpsProtocol += "RMC ";
    if (gps.course.isUpdated()) gpsProtocol += "VTG ";
    if (gps.satellites.isUpdated()) gpsProtocol += "GSV ";
    if (gpsProtocol.length() == 0) gpsProtocol = "NMEA";
    
    gpsProtocol += String(gpsBaudRate);
  } else {
    gpsProtocol = "No GPS data";
  }
  
  display.setCursor(0, 8);
  display.println(gpsProtocol);
  
  // Display Lat, Lng, and Sat on separate lines
  if (gps.location.isValid()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    
    // Lat on its own line
    display.setCursor(0, 16);
    display.print("Lat ");
    display.println(lat, 5);
    
    // Lng on its own line
    display.setCursor(0, 24);
    display.print("Lng ");
    display.println(lng, 5);
  } else {
    display.setCursor(0, 16);
    display.println("Lat No Fix");
    display.setCursor(0, 24);
    display.println("Lng No Fix");
  }
  
  // Display satellite count
  display.setCursor(0, 32);
  display.print("Sat ");
  display.println(gps.satellites.value());
  
  // Add NMEA stats in a simple format
  display.setCursor(0, 40);
  display.print("NMEA: OK=");
  display.print(gps.passedChecksum());
  display.print(" Err=");
  display.println(gps.failedChecksum());

  // Display altitude with correction if valid
  if (gps.altitude.isValid()) {
    float correctedAlt = gps.altitude.meters() + altitudeCorrection;
    display.setCursor(64, 32);
    display.print("Alt ");
    display.print(correctedAlt, 0);
    display.println("m");
  }

  // Display course and speed if valid
  if (gps.course.isValid() && gps.speed.isValid()) {
    display.setCursor(0, 48);
    display.print("Course: ");
    display.print(gps.course.deg());
    display.println("°");
    
    display.setCursor(0, 56);
    display.print("Speed: ");
    
    // Check if GPS data quality is good enough for reliable speed
    bool reliableSpeed = gps.satellites.value() >= 4 && gps.hdop.isValid() && gps.hdop.hdop() < 3.0;
    
    // Apply minimum threshold for speed to avoid erroneous readings
    float speedKmph = gps.speed.kmph();
    if (!reliableSpeed || speedKmph < 3.0) { // Below 3 km/h is likely noise
      speedKmph = 0;
    }
    
    display.print(speedKmph, 1);
    display.println(" km/h");
  }

  display.display();
}

// Function to display COMPASS STATUS on the OLED
void displayCompassLogOnOLED() {
  if (!display_initialized) return;

  display.clearDisplay(); // Clear the entire screen first
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Handle special calibration/declination states
  if (isSelectingCalibrationMode) {
    // --- Calibration Mode Selection UI ---
    display.setCursor(0, 0);
    display.println("Select Cal Mode:");
    display.println("---------------");
    
    // Display all modes, highlight the current one
    for (int i = 0; i < NUM_CALIBRATION_MODES; i++) {
      if (i == calibrationModeIndex) {
        display.print("> "); // Indicate current selection
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
    // --- Calibration Mode UI for QMC --- 
    display.setCursor(0, 0);
    display.println("-- Calibrating --");
    display.setCursor(0, 10);
    display.print("Mode: ");
    display.print(calibrationPoints);
    display.println("-point");
    
    display.setCursor(0, 26);
    String directions = "";
    
    // Get appropriate direction based on calibration step and mode
    if (calibrationPoints == 4) {
      // 4-point calibration: N, E, S, W
      switch (calibrationStep) {
        case 1: directions = "Point NORTH, Click"; break;
        case 2: directions = "Point EAST, Click"; break;
        case 3: directions = "Point SOUTH, Click"; break;
        case 4: directions = "Point WEST, Click"; break;
      }
    } 
    else if (calibrationPoints == 8) {
      // 8-point calibration: N, S, E, W, NE, SE, SW, NW
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
    else { // 16-point
      // 16-point: Full circle in 22.5° increments
      directions = "Point to " + String(calibrationStep * 22.5) + " deg";
    }
    
    display.println(directions);
    
    // Progress indicator
    display.setCursor(0, 46);
    display.print("Progress: ");
    display.print(calibrationStep - 1);
    display.print("/");
    display.println(calibrationPoints);
    
    display.setCursor(0, 56);
    display.println("(Hold Btn to Cancel)");
  } 
  else if (isSettingDeclination && activeCompass == &hmcCompass) {
    // --- Declination Setting UI for HMC ---
    display.setCursor(0, 0);
    display.println("- Set Declination -");
    
    // Instructions for pointing to true north
    display.setCursor(0, 16);
    display.println("Point device to TRUE");
    display.setCursor(0, 26);
    display.println("NORTH, then click");
    
    // Current compass reading
    display.setCursor(0, 40);
    display.print("Current: ");
    display.print(activeCompass->getAzimuth());
    display.println(" deg");
    
    // Controls
    display.setCursor(0, 56);
    display.println("Hold: Cancel");
  }
  else if (isSettingInversion) {
    // --- Compass Inversion Setting UI ---
    display.setCursor(0, 0);
    display.println("Invert Compass?");
    display.println("---------------");
    
    // Display current compass - make it smaller and centered
    int centerX = SCREEN_WIDTH / 2;
    int centerY = 30; // Position it more toward the top
    int radius = 15;  // Smaller radius to avoid overlap
    
    if (activeCompass != nullptr) {
      int heading = activeCompass->getAzimuth();
      
      // Apply current inversion setting for preview
      if (compassInverted) {
        heading = (heading + 180) % 360;
      }
      
      // Draw the compass
      float angleRad = radians(270 - heading);
      int endX = centerX + radius * cos(angleRad);
      int endY = centerY + radius * sin(angleRad);
      
      display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
      display.drawLine(centerX, centerY, endX, endY, SSD1306_WHITE);
      display.fillCircle(centerX, centerY, 2, SSD1306_WHITE);
      
      // Draw cardinal points - smaller and closer to the circle
      display.setCursor(centerX - 2, centerY - radius - 6);
      display.print("N");
      display.setCursor(centerX - 2, centerY + radius);
      display.print("S");
      display.setCursor(centerX + radius, centerY - 2);
      display.print("E");
      display.setCursor(centerX - radius - 5, centerY - 2);
      display.print("W");
      
      // Current heading - moved to the side
      display.setCursor(68, 14);
      display.print("Hdg:");
      display.print(heading);
      display.print("\xB0"); // Degree symbol
    }
    
    // Instructions - clear separation at bottom
    display.setCursor(0, SCREEN_HEIGHT - 18);
    display.println("Click: Normal");
    display.setCursor(70, SCREEN_HEIGHT - 18);
    display.println("Hold: Invert");
    
    // Show inversion status
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print("Current: ");
    display.print(compassInverted ? "Inverted" : "Normal");
  }
  else {
    // Normal Compass Status Screen
    // Center the title
    String title = "-- Compass Status --";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println(title);
    
    // Print Compass Status
    if (activeCompass != nullptr) {
      // Display compass type and heading
      display.setCursor(0, 10);
      display.print(activeCompass->getSensorName());
      
      // Show declination if using HMC5883L
      if (activeCompass == &hmcCompass) {
        float declDegrees = currentDeclination * 180.0 / PI;
        display.print(" Decl:");
        display.print(declDegrees, 1);
        display.print("\xB0"); // Degree symbol
      }
      
      // Display heading
      activeCompass->read();
      int displayHeading = activeCompass->getAzimuth();
      // Apply inversion if compass is inverted
      if (compassInverted) {
        displayHeading = (displayHeading + 180) % 360;
      }
      
      display.setCursor(0, 20);
      display.print("Az ");
      display.print(displayHeading);
      display.print("\xB0 "); // Degree symbol
      
      // Get cardinal direction
      char dirArray[4] = {' ', ' ', ' ', '\0'};
      activeCompass->getDirection(dirArray, displayHeading);
      display.print(dirArray);
      
      // Print raw sensor data
      display.setCursor(0, 30);
      display.print("X: ");
      display.println(activeCompass->getX());
      display.setCursor(0, 38);
      display.print("Y: ");
      display.println(activeCompass->getY());
      display.setCursor(0, 46);
      display.print("Z: ");
      display.println(activeCompass->getZ());
      
      if (compassInverted) {
        display.setCursor(0, 56);
        display.print("[Inverted]");
      }
    } else {
      display.setCursor(0, 20);
      display.println("No compass detected");
    }
    
    // Add calibration prompt in bottom right
    if (activeCompass != nullptr) {
      String calibPrompt = "";
      if (activeCompass == &qmcCompass) {
        calibPrompt = "Cal: Hold";
      } else if (activeCompass == &hmcCompass) {
        calibPrompt = "Decl: Hold";
      }
      
      // Calculate position to right-align text
      display.getTextBounds(calibPrompt, 0, 0, &x1, &y1, &w, &h);
      display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 8);
      display.print(calibPrompt);
    }
  }

  display.display();
}

// Function to display graphical compass and Lat/Lon
void displayGraphicCompass() {
  if (!display_initialized) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // --- Normal Graphic Compass Mode UI --- 
  // Compass Graphic - centered on screen
  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2; // Fully centered vertically
  int radius = 22; // Slightly larger radius
  int heading = 0;
  
  if (activeCompass != nullptr) {
    heading = activeCompass->getAzimuth();
    
    // Apply compass inversion if needed
    if (compassInverted) {
      heading = (heading + 180) % 360;
    }
    
    // Fix the compass display - invert the angle
    // North should be up (270 degrees in screen coordinates)
    float angleRad = radians(270 - heading); // This makes needle point to correct direction
    
    // Draw the compass circle
    display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
    
    // Draw the compass needle (pointy end indicates North)
    int endX = centerX + radius * cos(angleRad);
    int endY = centerY + radius * sin(angleRad);
    display.drawLine(centerX, centerY, endX, endY, SSD1306_WHITE);
    
    // Add arrowhead to make direction clearer
    float arrowAngle1 = angleRad + radians(150);
    float arrowAngle2 = angleRad + radians(210);
    int arrowLength = 6;
    int arrow1X = endX + arrowLength * cos(arrowAngle1);
    int arrow1Y = endY + arrowLength * sin(arrowAngle1);
    int arrow2X = endX + arrowLength * cos(arrowAngle2);
    int arrow2Y = endY + arrowLength * sin(arrowAngle2);
    display.drawLine(endX, endY, arrow1X, arrow1Y, SSD1306_WHITE);
    display.drawLine(endX, endY, arrow2X, arrow2Y, SSD1306_WHITE);
    
    // Draw center dot
    display.fillCircle(centerX, centerY, 2, SSD1306_WHITE);
    
    // Draw compass rose labels with improved positioning
    display.setCursor(centerX - 2, centerY - radius - 7); 
    display.print("N"); 
    display.setCursor(centerX - 2, centerY + radius + 1);
    display.print("S");
    display.setCursor(centerX + radius + 2, centerY - 3);
    display.print("E");
    display.setCursor(centerX - radius - 7, centerY - 3);
    display.print("W");
  } else {
    // Draw an error indicator
    display.setCursor(centerX - 8, centerY - 4); 
    display.print("ERR");
  }

  // --- Show azimuth as a number at the top --- 
  if (activeCompass != nullptr) {
    display.setCursor(0, 0);
    display.print("Az "); 
    display.print(heading);
    
    // Remove cardinal direction display from top right
  } else {
    display.setCursor(0, 0);
    display.print("Az ---");
  }
  
  // --- Display altitude in top right if available ---
  if (gps.altitude.isValid()) {
    // Check if GPS data quality is good enough for reliable altitude
    bool reliableAlt = gps.satellites.value() >= 5 && gps.hdop.isValid() && gps.hdop.hdop() < 2.5;
    
    // Get altitude and apply correction
    float correctedAlt = gps.altitude.meters() + altitudeCorrection;
    
    // Only display if reasonably reliable
    if (reliableAlt) {
      String altStr = "Alt " + String(correctedAlt, 0) + "m";
      int16_t x1, y1;
      uint16_t w, h;
      display.getTextBounds(altStr, 0, 0, &x1, &y1, &w, &h);
      display.setCursor(SCREEN_WIDTH - w, 0);
      display.print(altStr);
    }
  }
  
  // --- Display speed on the right side of compass if > 0 ---
  if (gps.speed.isValid()) {
    // Check if GPS data quality is good enough for reliable speed
    bool reliableSpeed = gps.satellites.value() >= 4 && gps.hdop.isValid() && gps.hdop.hdop() < 3.0;
    
    // Apply minimum threshold for speed to avoid erroneous readings
    float speedKmph = gps.speed.kmph();
    if (!reliableSpeed || speedKmph < 3.0) { // Below 3 km/h is likely noise
      speedKmph = 0;
    }
    
    // Only display if speed is greater than 0
    if (speedKmph > 0) {
      String spdStr = "Spd " + String(speedKmph, 1) + "km/h";
      int16_t x1, y1;
      uint16_t w, h;
      display.getTextBounds(spdStr, 0, 0, &x1, &y1, &w, &h);
      // Position to the right of the compass, vertically centered
      display.setCursor(centerX + radius + 4, centerY - h/2);
      display.print(spdStr);
    }
  }
  
  // --- Lat/Lon Data moved to bottom of screen ---
  if (gps.location.isValid()) {
    // Format coordinates to prevent overflow
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    
    // Display lat label on left
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.println("Lat");
    
    // Display lat value below the label
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print(String(lat, 5));
    
    // Display lng label aligned to right edge
    String lngLabel = "Lng";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(lngLabel, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 16);
    display.println(lngLabel);
    
    // Display lng value below its label, right aligned
    String lngValue = String(lng, 5);
    display.getTextBounds(lngValue, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 8);
    display.print(lngValue);
  } else {
    // No Fix case
    display.setCursor(0, SCREEN_HEIGHT - 16);
    display.println("Lat");
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.println("No Fix");
    
    // Right align Lng label for No Fix case
    String lngLabel = "Lng";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(lngLabel, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 16);
    display.println(lngLabel);
    
    // Right align No Fix message
    String noFix = "No Fix";
    display.getTextBounds(noFix, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - 8);
    display.println(noFix);
  }
  
  // Display inversion status indicator if inverted
  if (compassInverted) {
    display.setCursor(0, 0);
    display.print("[Inv]");
  }

  display.display();
}

// Function to display a world map with current position
void displayWorldMap() {
  if (!display_initialized) return;

  display.clearDisplay();
  
  // Draw the world map bitmap
  display.drawBitmap(0, 0, world_map, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  
  // Check if we have a valid GPS position
  if (gps.location.isValid()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    
    // Convert latitude and longitude to x,y coordinates on the 128x64 bitmap
    // For Mercator projection:
    // X coordinate - longitude is linear
    int posX = (int)((lng + 180.0) / 360.0 * SCREEN_WIDTH);
    
    // Y coordinate - latitude uses Mercator formula
    // Constrain latitude to avoid infinite values near poles
    if (lat > 85.0) lat = 85.0;
    if (lat < -85.0) lat = -85.0;
    
    // Mercator formula: y = ln(tan(pi/4 + lat*pi/360))
    float latRad = lat * PI / 180.0;
    float mercN = log(tan((PI/4) + (latRad/2)));
    // Scale to screen height (0 at top, SCREEN_HEIGHT at bottom)
    int posY = (int)(SCREEN_HEIGHT/2 - (mercN * SCREEN_HEIGHT / (2*PI)));
    
    // Make sure position is on screen
    posX = constrain(posX, 0, SCREEN_WIDTH-1);
    posY = constrain(posY, 0, SCREEN_HEIGHT-1);
    
    // Make the position blink (1/2 second on, 1/2 second off)
    if ((millis() / 500) % 2 == 0) {
      // Draw position marker (filled circle with outline)
      display.fillCircle(posX, posY, 3, SSD1306_WHITE);
      display.drawCircle(posX, posY, 4, SSD1306_WHITE);
    } else {
      // Draw just the outline when in "off" cycle for better visibility
      display.drawCircle(posX, posY, 3, SSD1306_WHITE);
      display.drawCircle(posX, posY, 4, SSD1306_WHITE);
    }
    
    // Optionally draw a small heading indicator if we have compass data
    if (activeCompass != nullptr) {
      activeCompass->read();
      int heading = activeCompass->getAzimuth();
      
      // Apply compass inversion if needed
      if (compassInverted) {
        heading = (heading + 180) % 360;
      }
      
      // Draw a small line indicating heading direction
      float radians = heading * PI / 180.0;
      int arrowLength = 8;
      int endX = posX + sin(radians) * arrowLength;
      int endY = posY - cos(radians) * arrowLength;
      
      display.drawLine(posX, posY, endX, endY, SSD1306_WHITE);
    }
    
    // Removed the coordinate display at the bottom
  } else {
    // No GPS fix - display a small message instead of full coordinates
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // Small indicator at the bottom for no fix
    display.setCursor(2, SCREEN_HEIGHT - 8);
    display.print("No Fix");
  }
  
  display.display();
}

// Helper function to scan I2C devices
void scanI2CDevices(void) {
  byte error, address;
  int nDevices = 0;
  
  logMessage("\nScanning I2C bus..."); // Use logMessage
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      String device_msg = "I2C device found at address 0x";
      if (address < 16) {
        device_msg += "0";
      }
      device_msg += String(address, HEX);
      
      // Identify common devices
      if (address == 0x3C || address == 0x3D) {
        device_msg += " (likely SSD1306 OLED display)";
      }
      else if (address == 0x0D) {
        device_msg += " (likely QMC5883L compass)";
      }
      else if (address == 0x1E) {
        device_msg += " (likely HMC5883L compass)";
      }
      else {
        device_msg += " (unknown device)";
      }
      logMessage(device_msg); // Log the combined message
      
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    logMessage("!!! WARNING: No I2C devices found !!!");
    logMessage("Check connections and power to I2C devices");
  } else {
    String count_msg = "Found " + String(nDevices) + " I2C device(s)";
    logMessage(count_msg); // Log the combined message
  }
  logMessage(""); // Log empty line
} 