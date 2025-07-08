#ifndef MAIN_GLOBALS_H
#define MAIN_GLOBALS_H

#include <TinyGPS++.h>
#include "CompassInterface.h"
#include <Preferences.h>
#include <vector>
#include <String>

#ifdef __cplusplus
extern "C" {
#endif

extern int SCREEN_WIDTH;
extern int SCREEN_HEIGHT;
extern const int NUM_CALIBRATION_MODES;
extern const int CALIBRATION_MODE_POINTS[];
extern const unsigned char world_map[];

#ifdef __cplusplus
}
#endif

// C++-only globals
extern TinyGPSPlus gps;
extern CompassInterface* activeCompass;
extern bool display_initialized;

// Add these for global preferences access
extern Preferences preferences;
extern const char* PREF_NAMESPACE;
extern const char* KEY_RADIO_USB_MODE;

// Add GSV handling globals
extern bool hasValidGSVs;
extern std::vector<String> gsvBuffer;
extern String latestGGA;
extern String latestRMC;
extern String latestGSA;

#endif // MAIN_GLOBALS_H 