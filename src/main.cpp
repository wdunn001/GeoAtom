#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "CompassInterface.h"
#include <vector>
#include <String>
#include <set>
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
#include <M5Unified.h> // Added M5Unified library
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
  // Empty stub - display functionality disabled
}

// Add this function to draw a save icon
void drawSaveIcon(U8G2 &display) {
  // Empty stub - display functionality disabled
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
#define GPS_RX 22 // ESP32 RX pin connected to BN-880 TX
#define GPS_TX 19  // ESP32 TX pin connected to BN-880 RX

// TTL to RS232 level converter connections (Optional Radio Output)
#define RADIO_RX 23  // ESP32 RX pin connected to TTL TX
#define RADIO_TX 33  // ESP32 TX pin connected to TTL RX
// --------------------------------------------------------------

// Create ICOM7100 configurator instance
ICOM7100Configurator* radioConfig = nullptr;

// Batched logging
static std::deque<String> log_batch;
static String lastMagnetoMsg = "";
static unsigned long lastLogFlush = 0;
const unsigned long LOG_FLUSH_INTERVAL = 2000; // ms

// --- Global NMEA sentence buffers for radio forwarding ---
String latestGGA = "";
String latestRMC = "";
String latestGSA = "";
std::vector<String> gsvBuffer; // Buffer for all GSV sentences in a cycle
bool hasValidGSVs = false;  // Add this variable for tracking valid GSV messages

void logMessage(const String& msg) {
    // Only keep the last magnetometer message in the batch
    if (msg.startsWith("Using magnetometer for heading")) {
        lastMagnetoMsg = msg;
    } else {
        log_batch.push_back(msg);
    }
    
    // Use M5 logging (needs conversion to c_str() for const char*)
    M5.Log.println(msg.c_str()); // Log using M5Unified
    
    // Keep Serial logging commented as it was before
    // Serial.println(msg); // Log to Serial
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
    
    // Initialize M5Unified before using its logging capabilities
    M5.begin();
    
    // Configure M5 logger (skipping unsupported methods)
    // M5.Log is automatically enabled with M5.begin()
    
    logMessage("[DEBUG] Entered setup()");
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    logMessage("[DEBUG] After Wire.begin");
    logMessage("Display disabled - skipping initialization");
    
    // Set display_initialized to false to disable display functionality
    display_initialized = false;
    
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
    logMessage("Initializing GPS module (UBlox M10)...");
    gpsBaudRate = 115200; // Set to 115200 for M100-5883 GPS
    bool gpsInitSuccess = false;
    // Basic GPS serial start needed before configuration
    GPS.begin(gpsBaudRate, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(100); // Small delay for serial port
    logMessage("[DEBUG] After GPS.begin");
    if (GPS) {
        logMessage("GPS Serial Port OK. Configuring...");
        delay(500);
        GPSConfigurator gpsConfig(GPS);
        logMessage("Initializing and configuring GPS module via GPSConfigurator::init()...");
        uint32_t detectedBaud = gpsConfig.autoDetectBaudRate();
        logMessage("Detected GPS baud rate: " + String(detectedBaud));
        delay(500);
        //gpsConfig.init();
        gpsInitSuccess = true;
        // Flush any pending data
        while (GPS.available()) {
            GPS.read();
        }
    } else {
        logMessage("*** ERROR: Failed to open GPS Serial Port! ***");
        gpsInitSuccess = false;
    }
    gpsInitialized = gpsInitSuccess;
    if (gpsInitialized) {
        logMessage("GPS initialization complete. Waiting for satellite fix...");
    } else {
        logMessage("*** WARNING: GPS initialization failed or incomplete. ***");
    }
    // --- End GPS Initialization ---
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
  
  // Only update display if it's initialized
  if (display_initialized) {
    updateDisplayForCurrentMode();
  }

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

  // Log compass info every 5 seconds
  static unsigned long lastCompassLog = 0;
  if (activeCompass != nullptr && millis() - lastCompassLog > 5000) {
    logMessage("Compass Info: " + activeCompass->getSensorInfoString());
    lastCompassLog = millis();
  }

  // Process any available GPS data
  if (gpsInitialized) {
    static String nmeaLine = "";
    static unsigned long gpsCharCount = 0;
    static unsigned long lastCharStatsTime = 0;
    static unsigned long charsPerSecond = 0;
    while (GPS.available()) {
      char c = GPS.read();
      gpsCharCount++;
      if (c == '\n' || c == '\r') {
        if (nmeaLine.startsWith("$") && nmeaLine.length() > 6) {
          logMessage("GPS RAW: " + nmeaLine);
          String type = nmeaLine.substring(3, 6);
          if (type == "GSV") {
            // Store GSV message in buffer for satellite data
            // Check if this is the start of a new GSV set
            int comma1 = nmeaLine.indexOf(',', 7); // after $GPGSV,
            int comma2 = nmeaLine.indexOf(',', comma1 + 1);
            int comma3 = nmeaLine.indexOf(',', comma2 + 1);
            
            if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
              int totalMsgs = nmeaLine.substring(7, comma1).toInt();
              int msgNum = nmeaLine.substring(comma1 + 1, comma2).toInt();
              int totalSats = nmeaLine.substring(comma2 + 1, comma3).toInt();
              
              // If this is the first message in a set, log it and clear buffer if it's a new set
              if (msgNum == 1) {
                logMessage("Starting new GSV set with " + String(totalMsgs) + " messages, " + String(totalSats) + " satellites");
                gsvBuffer.clear(); // Only clear the buffer at the start of a new set
              }
              
              // Always add the GSV message to the buffer, even if totalSats is 0
              // ICOM 7100 needs GSV messages even with 0 satellites for proper display
              gsvBuffer.push_back(nmeaLine);
              
              // Only forward complete GSV sets
              if (msgNum == totalMsgs) {
                logMessage("Completed GSV set with " + String(gsvBuffer.size()) + " messages");
                
                // Filter and improve GSV data for more consistent display
                if (gsvBuffer.size() > 0 && totalSats > 0) {
                  // Count how many satellites have actual signal strength values
                  int satellitesWithSignal = 0;
                  
                  // Process each GSV message to ensure it has the correct talker ID
                  // and count satellites with valid signal strength
                  for (int i = 0; i < gsvBuffer.size(); i++) {
                    String currentGSV = gsvBuffer[i];
                    
                    // Count satellites with signal strength in this message
                    int commaCount = 0;
                    bool hasSignalStrength = false;
                    
                    // Each satellite has 4 fields (PRN,elevation,azimuth,SNR)
                    // Check for valid signal strength values in SNR fields
                    for (int j = 0; j < currentGSV.length(); j++) {
                      if (currentGSV[j] == ',') {
                        commaCount++;
                        // Signal strength fields are the 4th, 8th, 12th, and 16th fields
                        if (commaCount == 7 || commaCount == 11 || commaCount == 15 || commaCount == 19) {
                          int nextComma = currentGSV.indexOf(',', j + 1);
                          int nextStar = currentGSV.indexOf('*', j + 1);
                          int endPos = (nextComma > 0) ? nextComma : nextStar;
                          
                          if (endPos > j + 1) {
                            String snrValue = currentGSV.substring(j + 1, endPos);
                            if (snrValue.length() > 0 && snrValue.toInt() > 0) {
                              satellitesWithSignal++;
                              hasSignalStrength = true;
                            }
                          }
                        }
                      }
                    }
                    
                    // If the talker ID isn't GP, we need to convert to GPGSV for ICOM compatibility
                    if (!currentGSV.startsWith("$GP")) {
                      // Keep the original prefix, don't convert
                    }
                  }
                  
                  // Only mark the dataset as valid if there are actual satellites with signal
                  if (satellitesWithSignal > 0) {
                    logMessage("Found " + String(satellitesWithSignal) + " satellites with signal strength");
                    hasValidGSVs = true;
                  } else {
                    logMessage("No satellites with signal strength in this GSV set");
                    hasValidGSVs = false;
                  }
                  
                  // Forward NMEA to radio once we have a complete GSV set with signal data
                  if (hasValidGSVs && radioConfig != nullptr) {
                    radioConfig->forwardNMEAToRadio(gps, altitudeCorrection, charsPerSecond);
                  }
                } else {
                  hasValidGSVs = false;
                  logMessage("No valid GSV satellites found, skipping set");
                }
              }
            } else {
              logMessage("Invalid GSV message format: " + nmeaLine);
            }
          }
          else if (type == "GGA") {
            // Patch the satellite count field to match gps.satellites.value()
            int firstComma = nmeaLine.indexOf(',');
            int field = 0, idx = 0, lastIdx = 0;
            String patchedGGA = "";
            while (field < 7 && idx != -1) {
              idx = nmeaLine.indexOf(',', lastIdx);
              if (idx == -1) break;
              patchedGGA += nmeaLine.substring(lastIdx, idx + 1);
              lastIdx = idx + 1;
              field++;
            }
            // Insert the correct satellite count
            patchedGGA += String(gps.satellites.value());
            // Find the next comma after the sat count field
            int nextComma = nmeaLine.indexOf(',', lastIdx);
            if (nextComma != -1) {
              // Append the rest of the GGA sentence after the sat count
              patchedGGA += nmeaLine.substring(nextComma);
            }
            // Recalculate and update the NMEA checksum
            int starIdx = patchedGGA.indexOf('*');
            String body = (starIdx != -1) ? patchedGGA.substring(1, starIdx) : patchedGGA.substring(1);
            uint8_t checksum = 0;
            for (size_t i = 0; i < body.length(); i++) checksum ^= body[i];
            String out = patchedGGA;
            if (starIdx != -1) {
              out = patchedGGA.substring(0, starIdx + 1);
            } else {
              out += "*";
            }
            if (checksum < 16) out += "0";
            out += String(checksum, HEX);
            // If the original had \r\n, preserve it
            if (nmeaLine.endsWith("\r\n")) out += "\r\n";
            else if (nmeaLine.endsWith("\n")) out += "\n";
            else if (nmeaLine.endsWith("\r")) out += "\r";
            latestGGA = out;
          }
          else if (type == "RMC") latestRMC = nmeaLine;
          else if (type == "GSA") {
            // Simply store the most recent GSA message
            // We'll convert it to $GPGSA if needed
            if (nmeaLine.startsWith("$")) {
              if (!nmeaLine.startsWith("$GP")) {
                // No longer converting talker ID to GP for ICOM compatibility
                latestGSA = nmeaLine;
              } else {
                // Already has GP prefix, just use as is
                latestGSA = nmeaLine;
              }
              
              // Log that we received a GSA message
              logMessage("Updated latest GSA message");
            }
          }
        }
        nmeaLine = "";
      } else {
        nmeaLine += c;
      }
      if (gps.encode(c)) {
        // Debug when a complete sentence is successfully parsed
        static unsigned long lastGPSDebugTime = 0;
        if (millis() - lastGPSDebugTime > 5000) { // Debug GPS data every 5 seconds
          lastGPSDebugTime = millis();
          // Log GPS data status
          String debug = "GPS Data: ";
          debug += "Valid: " + String(gps.location.isValid() ? "Yes" : "No");
          debug += ", Lat: " + String(gps.location.lat(), 6);
          debug += ", Lng: " + String(gps.location.lng(), 6);
          debug += ", Alt: " + String(gps.altitude.meters(), 1);
          debug += ", Sats: " + String(gps.satellites.value());
          logMessage(debug);
          // Additional debug for backup data handling
          if (radioConfig && radioConfig->isUsingBackupData()) {
            logMessage("Using backup data. Packets may not be processed by TinyGPS++");
          }
        }
      }
    }
    // Update charsPerSecond every second
    if (millis() - lastCharStatsTime > 1000) {
      charsPerSecond = gpsCharCount;
      gpsCharCount = 0;
      lastCharStatsTime = millis();
    }
    
    // Forward NMEA messages based on configuration
    if (radioConfig != nullptr) {
      // Always forward to radio
      radioConfig->forwardNMEAToRadio(gps, altitudeCorrection, charsPerSecond);
      
      // If we're using backup data, capture recent messages and feed them to TinyGPS++
      static bool lastBackupStatus = false;
      
      if (radioConfig->isUsingBackupData()) {
        static unsigned long lastBackupFeedTime = 0;
        
        if (!lastBackupStatus) {
          // Log when we first switch to backup mode
          logMessage("Switching to GPS backup data mode");
          lastBackupStatus = true;
        }
        
        if (millis() - lastBackupFeedTime > 1000) { // Process backup data every second
          lastBackupFeedTime = millis();
          
          // Get the last generated backup messages from radioConfig
          String lastGGA = radioConfig->getLastGeneratedGGA();
          String lastRMC = radioConfig->getLastGeneratedRMC();
          
          // Feed these messages to TinyGPS++ so it can update its state
          if (lastGGA.length() > 0) {
            for (size_t i = 0; i < lastGGA.length(); i++) {
              gps.encode(lastGGA[i]);
            }
          }
          
          if (lastRMC.length() > 0) {
            for (size_t i = 0; i < lastRMC.length(); i++) {
              gps.encode(lastRMC[i]);
            }
          }
        }
      } else {
        // Reset backup status flag when we're not in backup mode
        if (lastBackupStatus) {
          logMessage("Returning to normal GPS data mode");
          lastBackupStatus = false;
        }
      }
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







