#include "yuma_http_service.h"
#include <HTTPClient.h>
#include <WiFi.h>

// List of Yuma almanac URLs to try in order
const char* YUMA_URLS[] = {
    "https://celestrak.org/GPS/almanac/Yuma/almanac.yuma.txt",
    "https://www.navcen.uscg.gov/sites/default/files/gps/almanac/current_yuma.txt"
};
const int NUM_YUMA_URLS = sizeof(YUMA_URLS) / sizeof(YUMA_URLS[0]);

void initYumaHttpService() {
    // Placeholder for any initialization needed
}

bool downloadYumaAlmanac() {
    if (WiFi.status() != WL_CONNECTED) return false;
    for (int i = 0; i < NUM_YUMA_URLS; ++i) {
        HTTPClient http;
        http.begin(YUMA_URLS[i]);
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            // TODO: Save or parse the almanac data
            String payload = http.getString();
            // Save to file or process as needed
            http.end();
            return true;
        }
        http.end();
    }
    return false;
} 