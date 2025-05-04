#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "CompassInterface.h"
#include <vector>
#include <String>
#include <Preferences.h>
#include "GPS_Configurator.h"
#include "ICOM7100Configurator.h"  // Add new include
#include <map> // Add for std::map
#include "main_globals.h"
#include <Wire.h>
#include <SPI.h>
#include <algorithm>  // Add this for min function
#include <deque>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <DNSServer.h>
#include "radio_web_server.h"
#include "display_world_map.h"
#include "display_compass_status.h"
#include "display_manager.h"
#include "display_graphic_compass.h"
#include "display_gps_status.h"
#include "display_log.h"
#include "display_wifi_status.h"
#include <U8g2lib.h>
#include "ButtonHandler.h"
#include "yuma_http_service.h"
extern bool wifiSetupEnabled;
extern void displayWiFiStatus();
extern void setupWiFiAndWeb();

// Remove #define SCREEN_WIDTH and #define SCREEN_HEIGHT
// Instead, define them as variables for external linkage
int SCREEN_WIDTH = 128;
int SCREEN_HEIGHT = 64;



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
void drawPrivacyIndicator(U8G2 &display) {
  extern bool privacyModeEnabled;
  if (privacyModeEnabled) {
    int x = display.getWidth() - 10;
    int y = 2;
    // Lock body
    display.drawBox(x, y + 3, 8, 6);
    // Lock shackle
    display.drawFrame(x + 1, y, 6, 4);
    // Optional: "P" for privacy (draw a simple P shape)
    display.setDrawColor(0);
    display.drawPixel(x + 2, y + 5);
    display.drawPixel(x + 3, y + 4);
    display.drawPixel(x + 4, y + 5);
    display.drawPixel(x + 3, y + 6);
    display.drawPixel(x + 2, y + 7);
    display.setDrawColor(1);
  }
}

// Add this function to draw a save icon
void drawSaveIcon(U8G2 &display) {
  int x = display.getWidth() - 10;
  int y = 2;
  // Disk body
  display.drawBox(x, y, 8, 8);
  // Disk label
  display.setDrawColor(0);
  display.drawFrame(x + 2, y + 2, 4, 4);
  display.setDrawColor(1);
  // Disk shutter
  display.drawFrame(x - 1, y + 1, 2, 6);
}

// GPS setup
TinyGPSPlus gps;
HardwareSerial GPS(1);  // Using UART1 for GPS
HardwareSerial Radio(2); // Using UART2 for NMEA forwarding to Radio/TTL converterUsbRadio usbRadio(Radio);
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
 //   Serial.println(msg); // Log to Serial
}

void flushLogMessages() {
    // Flush all batched messages
    while (!log_batch.empty()) {
        Serial.println(log_batch.front().c_str());
        log_batch.pop_front();
    }
    // Flush the last magnetometer message (if any)
    if (!lastMagnetoMsg.isEmpty()) {
        Serial.println(lastMagnetoMsg.c_str());
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

// Replace Adafruit_SSD1306 display instance with U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ I2C_SCL, /* data=*/ I2C_SDA);

ButtonHandler buttonHandler;

bool ignoreNextRelease = false;

void setup() {
    //Serial.begin(115200);
    //delay(100); // Give time for USB CDC to initialize
    logMessage("[DEBUG] Entered setup()");
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    logMessage("[DEBUG] After Wire.begin");
    logMessage("Initializing display...");
    if (!display.begin()) {
        logMessage("SSD1306 allocation failed");
    } else {
        display_initialized = true;
        display.clearBuffer();
        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(0, 10, "System Ready");
        display.sendBuffer();
    }
    display_initialized = true;
    display.clearBuffer();
    display.setFont(u8g2_font_ncenB08_tr);
    display.drawStr(0, 10, "System Ready");
    display.sendBuffer();
    logMessage("[DEBUG] After display init");
    // Initialize Radio serial for NMEA data only
    Radio.begin(9600, SERIAL_8N1, RADIO_RX, RADIO_TX);
    logMessage("[DEBUG] After Radio.begin");
    // Initialize ICOM7100 configurator
    radioConfig = new ICOM7100Configurator(Radio);
    radioConfig->initialize();
    logMessage("[DEBUG] After radioConfig->initialize()");
    logMessage("System initializing...");
    // --- Initialize GPS with Configuration --- 
    logMessage("Initializing GPS module...");
    gpsBaudRate = 9600; // Set to 9600 baud to match radio
    bool gpsInitSuccess = false;
    // Basic GPS serial start needed before configuration
    GPS.begin(gpsBaudRate, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(100); // Small delay for serial port
    logMessage("[DEBUG] After GPS.begin");
    if (GPS) {
        logMessage("GPS Serial Port OK. Configuring NMEA...");
        GPSConfigurator gpsConfig(GPS);
        if (!gpsConfig.setBaudRate(gpsBaudRate)) {
            logMessage("Failed to set GPS baud rate (continuing)");
        }
        delay(GPS_CONFIG_RETRY_DELAY);
        if (!gpsConfig.setUpdateRateHz(1)) {
            logMessage("Failed to set GPS update rate (continuing)");
        }
        delay(GPS_CONFIG_RETRY_DELAY);
        if (!gpsConfig.setDynamicModel(0)) {
            logMessage("Failed to set GPS dynamic model (continuing)");
        }
        delay(GPS_CONFIG_RETRY_DELAY);
        logMessage("Attempting to enable NMEA Messages...");
        const uint8_t nmeaClass = 0xF0;
        const uint8_t msgIds[] = {0x00, 0x04, 0x05, 0x02, 0x03, 0x01};
        const char* msgNames[] = {"GGA", "RMC", "VTG", "GSA", "GSV", "GLL"};
        bool anyNmeaFail = false;
        for (size_t i = 0; i < sizeof(msgIds) / sizeof(msgIds[0]); ++i) {
            logMessage("  Sending enable command for: " + String(msgNames[i]) + "...");
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
    logMessage("[DEBUG] After GPS init");
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
    logMessage("[DEBUG] After compass init");
    // Load preferences OR apply defaults
    preferences.begin(PREF_NAMESPACE, true);
    useM5StackSmoothing = preferences.getBool(KEY_USE_M5_SMOOTHING, true);
    useM5StackInterference = preferences.getBool(KEY_USE_M5_INTERFERENCE, true);
    radioUsbMode = preferences.getBool(KEY_RADIO_USB_MODE, false);
    compassInverted = preferences.getBool(KEY_COMPASS_INVERTED, false);
    altitudeCorrection = preferences.getInt(KEY_ALT_CORRECTION, 0);
    if (activeCompass == &hmcCompass) {
        currentDeclination = preferences.getFloat(KEY_DECLINATION, 0.0f);
        hmcCompass.setDeclination(currentDeclination);
    }
    preferences.end();
    logMessage("[DEBUG] After preferences");
    currentDisplayMode = DisplayMode::GRAPHIC_COMPASS;
    logMessage("--- Active Settings ---");
    logMessage("M5 Smoothing: " + String(useM5StackSmoothing ? "ON" : "OFF"));
    logMessage("M5 Interference: " + String(useM5StackInterference ? "ON" : "OFF"));
    logMessage("-----------------------");
    display.clearBuffer();
    display.setFont(u8g2_font_ncenB08_tr);
    display.drawStr(0, 10, "System Ready");
    display.sendBuffer();
    delay(1000);
    logMessage("[DEBUG] After display ready message");
    setupButtonHandlers();
    logMessage("[DEBUG] After setupButtonHandlers");

    if (qmcCompass.begin()) {
        logMessage("QMC5883L initialized successfully.");
        activeCompass = &qmcCompass;
        preferences.begin(PREF_NAMESPACE, true);
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
        }
    }
    logMessage("[DEBUG] End of setup()");
}

void loop() {
  buttonHandler.update();
  UIState currentUIState = getCurrentUIState();
  updateDisplayForCurrentMode();

  // Handle button presses based on current UI state
  if (buttonHandler.wasLongPress()) {
    if (buttonHandlerMap[currentUIState].longPressHandler) {
      buttonHandlerMap[currentUIState].longPressHandler();
      ignoreNextRelease = true; // Set flag to ignore next release
    }
    waitingForDoubleClick = false; // Reset double click state
  }
  else if (buttonHandler.wasShortPress()) {
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







