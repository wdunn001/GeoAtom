#include "yuma_http_service.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>

// List of Yuma almanac URLs to try in order
const char* YUMA_URLS[] = {
    "https://celestrak.org/GPS/almanac/Yuma/almanac.yuma.txt",
    "https://www.navcen.uscg.gov/sites/default/files/gps/almanac/current_yuma.txt"
};
const int NUM_YUMA_URLS = sizeof(YUMA_URLS) / sizeof(YUMA_URLS[0]);
const char* YUMA_FILE_PATH = "/gps_almanac/current_yuma.txt";

void initYumaHttpService() {
    // Placeholder for any initialization needed
}

// Save the downloaded Yuma almanac to SPIFFS
bool downloadYumaAlmanac() {
    if (WiFi.status() != WL_CONNECTED) return false;
    if (!SPIFFS.begin(true)) return false;
    File file = SPIFFS.open(YUMA_FILE_PATH, FILE_WRITE);
    if (!file) return false;
    for (int i = 0; i < NUM_YUMA_URLS; ++i) {
        HTTPClient http;
        http.begin(YUMA_URLS[i]);
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            file.print(payload);
            file.close();
            http.end();
            return true;
        }
        http.end();
    }
    file.close();
    return false;
}

// Parse the GPS week from the Yuma file
int getYumaFileWeek() {
    if (!SPIFFS.begin(true)) return -1;
    File file = SPIFFS.open(YUMA_FILE_PATH, FILE_READ);
    if (!file) return -1;
    String line;
    while (file.available()) {
        line = file.readStringUntil('\n');
        if (line.startsWith("week:")) {
            int week = line.substring(line.indexOf(":") + 1).toInt();
            file.close();
            return week;
        }
    }
    file.close();
    return -1;
}

// Get current GPS week (approximate, based on Unix time)
int getCurrentGpsWeek() {
    // GPS epoch: Jan 6, 1980
    const time_t gpsEpoch = 315964800;
    time_t now = time(nullptr);
    int totalSec = (int)(now - gpsEpoch + 18); // Add leap seconds
    return totalSec / (7 * 24 * 3600);
}

// Check if the Yuma file is for the current week
bool isYumaFileCurrent() {
    int fileWeek = getYumaFileWeek();
    int currentWeek = getCurrentGpsWeek();
    return (fileWeek == currentWeek);
}

// Ensure the Yuma file is up-to-date (only if WiFi is connected)
bool ensureYumaAlmanacCurrent() {
    if (WiFi.status() != WL_CONNECTED) return false;
    if (!SPIFFS.begin(true)) return false;
    if (!SPIFFS.exists(YUMA_FILE_PATH) || !isYumaFileCurrent()) {
        return downloadYumaAlmanac();
    }
    return true;
}

bool hasInternetAccess() {
    HTTPClient http;
    http.setTimeout(3000); // 3 seconds
    http.begin(YUMA_URLS[0]);
    int httpCode = http.GET();
    http.end();
    return (httpCode == HTTP_CODE_OK);
} 