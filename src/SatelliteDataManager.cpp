#include "ICOM7100Configurator.h"
#include <algorithm>

SatelliteDataManager::SatelliteDataManager() 
    : lastFullUpdate(0), lastTransmitTime(0) {
    // Changing the timeout in the constructor instead of in the class definition
    // to ensure it takes effect
    SAT_TIMEOUT = 180000; // 3 minutes timeout (up from 30 seconds)
    UPDATE_INTERVAL = 5000; // Keep 5 seconds update interval
}

bool SatelliteDataManager::processGSVMessage(const String& gsvMessage) {
    // Format: $GPGSV,total_msgs,msg_num,total_sats,...
    // Each satellite then uses 4 fields: PRN,elevation,azimuth,SNR
    
    // Only process NMEA messages with proper format
    if (!gsvMessage.startsWith("$") || gsvMessage.length() < 10) {
        return false;
    }
    
    // Check which GNSS constellation this message is from (GPS, GLONASS, Galileo, BeiDou, etc.)
    String constellation = gsvMessage.substring(1, 3);
    
    // Extract key fields from the GSV message
    int talkerPos = gsvMessage.indexOf(',');
    if (talkerPos < 4) return false; // Not enough characters for proper prefix
    
    int comma1 = gsvMessage.indexOf(',', 7); // After $xxGSV,
    if (comma1 < 0) return false;
    
    int comma2 = gsvMessage.indexOf(',', comma1 + 1);
    if (comma2 < 0) return false;
    
    int comma3 = gsvMessage.indexOf(',', comma2 + 1);
    if (comma3 < 0) return false;
    
    // Extract message information
    int totalMsgs = gsvMessage.substring(7, comma1).toInt();
    int msgNum = gsvMessage.substring(comma1 + 1, comma2).toInt();
    int totalSats = gsvMessage.substring(comma2 + 1, comma3).toInt();
    
    // If this is the first message in the set, start tracking a full update
    if (msgNum == 1) {
        // We're starting a new GSV sequence
        if (totalSats > 0) {
            // If this is a valid message with satellites, mark as full update
            lastFullUpdate = millis();
        }
    }
    
    // If no satellites in the message or the total is zero, skip it
    if (totalSats <= 0) {
        return false;
    }
    
    // Find satellite data fields (each satellite has PRN,elevation,azimuth,SNR)
    int startPos = comma3 + 1;
    int fieldPos = startPos;
    
    // Each satellite entry has 4 fields: PRN,elevation,azimuth,SNR
    for (int i = 0; i < 4; i++) { // Max 4 satellites per GSV message
        int prn = -1, elevation = -1, azimuth = -1, snr = -1;
        
        // Parse PRN
        int nextComma = gsvMessage.indexOf(',', fieldPos);
        int nextStar = gsvMessage.indexOf('*', fieldPos);
        int endPos = (nextComma >= 0 && (nextStar < 0 || nextComma < nextStar)) ? nextComma : nextStar;
        
        if (endPos < 0 || endPos <= fieldPos) {
            break; // No more fields
        }
        
        String prnStr = gsvMessage.substring(fieldPos, endPos);
        if (prnStr.length() > 0) {
            prn = prnStr.toInt();
        }
        
        fieldPos = endPos + 1;
        if (fieldPos >= gsvMessage.length() || gsvMessage[fieldPos-1] == '*') {
            break; // End of message
        }
        
        // Parse elevation
        nextComma = gsvMessage.indexOf(',', fieldPos);
        nextStar = gsvMessage.indexOf('*', fieldPos);
        endPos = (nextComma >= 0 && (nextStar < 0 || nextComma < nextStar)) ? nextComma : nextStar;
        
        if (endPos < 0 || endPos <= fieldPos) {
            break; // No more fields
        }
        
        String elevStr = gsvMessage.substring(fieldPos, endPos);
        if (elevStr.length() > 0) {
            elevation = elevStr.toInt();
        }
        
        fieldPos = endPos + 1;
        if (fieldPos >= gsvMessage.length() || gsvMessage[fieldPos-1] == '*') {
            break; // End of message
        }
        
        // Parse azimuth
        nextComma = gsvMessage.indexOf(',', fieldPos);
        nextStar = gsvMessage.indexOf('*', fieldPos);
        endPos = (nextComma >= 0 && (nextStar < 0 || nextComma < nextStar)) ? nextComma : nextStar;
        
        if (endPos < 0 || endPos <= fieldPos) {
            break; // No more fields
        }
        
        String azStr = gsvMessage.substring(fieldPos, endPos);
        if (azStr.length() > 0) {
            azimuth = azStr.toInt();
        }
        
        fieldPos = endPos + 1;
        if (fieldPos >= gsvMessage.length() || gsvMessage[fieldPos-1] == '*') {
            break; // End of message
        }
        
        // Parse SNR
        nextComma = gsvMessage.indexOf(',', fieldPos);
        nextStar = gsvMessage.indexOf('*', fieldPos);
        endPos = (nextComma >= 0 && (nextStar < 0 || nextComma < nextStar)) ? nextComma : nextStar;
        
        if (endPos < 0 || endPos <= fieldPos) {
            // This might be the last field, so check if we're at the end of the string
            if (fieldPos < gsvMessage.length()) {
                String snrStr = gsvMessage.substring(fieldPos);
                int starPos = snrStr.indexOf('*');
                if (starPos >= 0) {
                    snrStr = snrStr.substring(0, starPos);
                }
                if (snrStr.length() > 0) {
                    snr = snrStr.toInt();
                }
            }
        } else {
            String snrStr = gsvMessage.substring(fieldPos, endPos);
            if (snrStr.length() > 0) {
                snr = snrStr.toInt();
            }
        }
        
        // Update satellite information if we have a valid PRN
        if (prn > 0) {
            // Create a unique ID that combines constellation and PRN
            // This ensures we don't confuse PRN 1 from GPS with PRN 1 from GLONASS
            int uniqueId = prn;
            
            // For satellites from BeiDou, Galileo, etc., add an offset to avoid collision
            // with GPS PRNs (which we want to keep as their original numbers for the ICOM)
            // We'll use simple offsets - GPS: 0, GLONASS: 100, Galileo: 200, BeiDou: 300, etc.
            if (constellation == "GB") {
                uniqueId = prn + 100; // BeiDou
            } else if (constellation == "GA") {
                uniqueId = prn + 200; // Galileo
            } else if (constellation == "GL") {
                uniqueId = prn + 300; // GLONASS
            } else if (constellation == "GQ") {
                uniqueId = prn + 400; // QZSS
            } else if (constellation == "GI") {
                uniqueId = prn + 500; // IRNSS/NavIC
            }
            
            // If a satellite is in our usedPRNs list, and this message has its data,
            // make sure to prioritize it
            bool isUsedForFix = std::find(usedPRNs.begin(), usedPRNs.end(), prn) != usedPRNs.end();
            
            // If we already know about this satellite, update its data
            // even if SNR is missing - we'll keep the last known SNR
            if (satellites.find(uniqueId) != satellites.end()) {
                // Update elevation and azimuth even if SNR is missing
                if (elevation >= 0) {
                    satellites[uniqueId].elevation = elevation;
                }
                if (azimuth >= 0) {
                    satellites[uniqueId].azimuth = azimuth;
                }
                
                // Only update SNR if it's valid
                if (snr > 0) {
                    satellites[uniqueId].snr = snr;
                }
                
                // If this satellite is used for fix, maintain that status
                if (isUsedForFix) {
                    satellites[uniqueId].used = true;
                }
                
                // Store the original PRN for proper display
                satellites[uniqueId].prn = prn;
                
                // Always update lastSeen time to prevent expiration
                satellites[uniqueId].lastSeen = millis();
            } 
            // For new satellites, we'll be more permissive - add them even with zero SNR
            // but set a minimal SNR so they show up
            else {
                // If SNR is missing or zero, assume a minimal value to ensure it's displayed
                if (snr <= 0) {
                    snr = 10; // Minimal SNR - ensures satellite appears on map
                }
                satellites[uniqueId] = SatelliteInfo(prn, elevation, azimuth, snr);
                
                // If this satellite is used for fix, mark it
                if (isUsedForFix) {
                    satellites[uniqueId].used = true;
                }
            }
        }
        
        fieldPos = endPos + 1;
        if (fieldPos >= gsvMessage.length() || gsvMessage[fieldPos-1] == '*') {
            break; // End of message
        }
    }
    
    return true;
}

bool SatelliteDataManager::processGSAMessage(const String& gsaMessage) {
    // Format: $GPGSA,mode1,mode2,sat1,sat2,...,sat12,pdop,hdop,vdop*CS
    
    // Only process NMEA messages with proper format
    if (!gsaMessage.startsWith("$") || gsaMessage.length() < 10) {
        return false;
    }
    
    // Extract PRNs of satellites used for fix from GSA message
    int modeField = gsaMessage.indexOf(',', 6) + 1; // Start after $xxGSA,
    if (modeField < 1) return false;
    
    int mode2Field = gsaMessage.indexOf(',', modeField) + 1;
    if (mode2Field < 1) return false;
    
    // Get fix type (1=none, 2=2D, 3=3D)
    int fix = gsaMessage.substring(mode2Field, gsaMessage.indexOf(',', mode2Field)).toInt();
    
    // Only clear PRN list if this is a valid message with a valid fix type
    // and if we don't have any satellites yet
    if (fix > 0 && usedPRNs.empty()) {
        usedPRNs.clear();
    }
    
    // If we don't have a fix, still continue to process - we may have PRNs
    // that we want to keep (more stable display)
    std::vector<int> newUsedPRNs;
    
    // Get satellite PRNs
    int satField = gsaMessage.indexOf(',', mode2Field) + 1;
    if (satField < 1) return false;
    
    // Read up to 12 satellite PRNs
    for (int i = 0; i < 12; i++) {
        int nextComma = gsaMessage.indexOf(',', satField);
        if (nextComma < 0) break; // No more fields
        
        String prnStr = gsaMessage.substring(satField, nextComma);
        if (prnStr.length() > 0) {
            int prn = prnStr.toInt();
            if (prn > 0) {
                newUsedPRNs.push_back(prn);
                
                // We need to check all constellation variants of this PRN
                // since we don't know which constellation it belongs to
                bool foundInDatabase = false;
                
                // Try to find this PRN in our satellite database
                for (auto& pair : satellites) {
                    // Check if this is the same PRN (original PRN, not the uniqueId)
                    if (pair.second.prn == prn) {
                        pair.second.used = true;
                        pair.second.lastSeen = millis();
                        foundInDatabase = true;
                    }
                }
                
                // If not found, create a placeholder entry
                if (!foundInDatabase) {
                    // Create a simple entry using the original PRN as uniqueId (GPS assumption)
                    satellites[prn] = SatelliteInfo(prn, 0, 0, 15);
                    satellites[prn].used = true;
                }
            }
        }
        
        satField = nextComma + 1;
    }
    
    // Only update the usedPRNs if we have at least one valid PRN in the message
    // This prevents empty messages from clearing our PRN list
    if (!newUsedPRNs.empty()) {
        // If this message has fewer PRNs than our current list, don't update unless there's a big difference
        // This helps maintain stability in the display
        if (newUsedPRNs.size() < usedPRNs.size() && 
            usedPRNs.size() - newUsedPRNs.size() < 3) {
            // Just keep our current list - small reduction isn't worth updating
            // But still add any new PRNs that weren't already in our list
            for (int newPrn : newUsedPRNs) {
                if (std::find(usedPRNs.begin(), usedPRNs.end(), newPrn) == usedPRNs.end()) {
                    usedPRNs.push_back(newPrn);
                }
            }
        } else {
            // Significant change or increase in PRNs, update our list
            usedPRNs = newUsedPRNs;
        }
    }
    
    return true;
}

std::vector<String> SatelliteDataManager::generateGSVMessages() {
    std::vector<String> messages;
    
    // Clean up expired satellites first
    cleanup();
    
    // First collect satellites used for position fix - these go in the first message
    std::vector<SatelliteInfo> positionFixSats;
    // Then collect other satellites with decent signal strength
    std::vector<SatelliteInfo> regularSats;
    
    for (const auto& pair : satellites) {
        if (pair.second.used) {
            // This satellite is used for position fix - highest priority
            positionFixSats.push_back(pair.second);
        } 
        else if (pair.second.snr >= 15) {
            // This satellite has reasonable signal but isn't used for fix
            regularSats.push_back(pair.second);
        }
        else {
            // Only include low-signal satellites if we have space
            if (positionFixSats.size() + regularSats.size() < 12) {
                regularSats.push_back(pair.second);
            }
        }
    }
    
    // Don't proceed if no satellites at all
    if (positionFixSats.empty() && regularSats.empty()) {
        return messages;
    }
    
    // Sort the position fix satellites by signal strength
    std::sort(positionFixSats.begin(), positionFixSats.end(), 
              [](const SatelliteInfo& a, const SatelliteInfo& b) {
                  return a.snr > b.snr;
              });
    
    // Sort the regular satellites by signal strength
    std::sort(regularSats.begin(), regularSats.end(), 
              [](const SatelliteInfo& a, const SatelliteInfo& b) {
                  return a.snr > b.snr;
              });
    
    // Combine the lists, with position fix satellites first
    std::vector<SatelliteInfo> allSats;
    allSats.insert(allSats.end(), positionFixSats.begin(), positionFixSats.end());
    allSats.insert(allSats.end(), regularSats.begin(), regularSats.end());
    
    // Limit to 16 satellites at most (4 GSV messages)
    if (allSats.size() > 16) {
        allSats.resize(16);
    }
    
    // Log the satellites being shown, especially those used for fix
    String debugMsg = "Satellites: ";
    for (const auto& sat : positionFixSats) {
        debugMsg += String(sat.prn) + "(used:" + String(sat.snr) + ") ";
    }
    debugMsg += "| Other: ";
    for (const auto& sat : regularSats) {
        // No need to check if it's in allSats since we're just logging
        debugMsg += String(sat.prn) + "(" + String(sat.snr) + ") ";
    }
    logMessage(debugMsg);
    
    // Calculate number of GSV messages needed (4 satellites per message)
    int totalSats = allSats.size();
    int totalMsgs = (totalSats + 3) / 4;
    
    // Each GSV message can contain up to 4 satellites
    for (int i = 0; i < totalMsgs; i++) {
        String gsv = "$GPGSV," + String(totalMsgs) + "," + String(i + 1) + "," + String(totalSats);
        
        // Add satellite data for this message (4 satellites max)
        int startSat = i * 4;
        int endSat = std::min(startSat + 3, totalSats - 1);
        
        for (int j = startSat; j <= endSat; j++) {
            const SatelliteInfo& sat = allSats[j];
            
            // Use the original PRN for display (not the uniqueId which may have offsets)
            gsv += "," + String(sat.prn);
            gsv += "," + String(sat.elevation);
            gsv += "," + String(sat.azimuth);
            
            // Ensure a minimum SNR value for display (even if it's 0)
            // For satellites used in position fix, ensure they have higher SNR
            int displaySnr = (sat.snr > 0) ? sat.snr : 10;
            if (sat.used && displaySnr < 30) {
                displaySnr = 30; // Ensure used satellites have very good signal display
            }
            gsv += "," + String(displaySnr);
        }
        
        // Calculate checksum
        uint8_t checksum = 0;
        for (size_t j = 1; j < gsv.length(); j++) {
            checksum ^= gsv[j];
        }
        
        // Add checksum
        gsv += "*";
        if (checksum < 16) gsv += "0";
        gsv += String(checksum, HEX);
        gsv.toUpperCase();
        
        // Add proper line ending for NMEA messages
        gsv += "\r\n";
        
        messages.push_back(gsv);
    }
    
    return messages;
}

String SatelliteDataManager::generateGSAMessage() {
    // Format: $GPGSA,mode1,mode2,sat1,sat2,...,sat12,pdop,hdop,vdop*CS
    
    // Clean up expired satellites first
    cleanup();
    
    // Get used satellites that still have valid data
    std::vector<int> currentUsedPRNs;
    for (int prn : usedPRNs) {
        if (satellites.find(prn) != satellites.end() && 
            (millis() - satellites[prn].lastSeen) < SAT_TIMEOUT) {
            currentUsedPRNs.push_back(prn);
        }
    }
    
    // If we don't have any used satellites, return an empty string
    if (currentUsedPRNs.empty()) {
        return "";
    }
    
    // Sort PRNs to ensure consistent order
    std::sort(currentUsedPRNs.begin(), currentUsedPRNs.end());
    
    // Build the GSA message
    String gsa = "$GPGSA,A,3"; // Auto selection, 3D fix
    
    // Add satellite PRNs (up to 12)
    for (int i = 0; i < 12; i++) {
        if (i < currentUsedPRNs.size()) {
            gsa += "," + String(currentUsedPRNs[i]);
        } else {
            gsa += ","; // Empty field for unused slots
        }
    }
    
    // Determine PDOP, HDOP, VDOP based on visible satellite count
    // Better geometry when more satellites are visible
    float pdop = 2.5, hdop = 1.5, vdop = 2.0;
    int usedCount = currentUsedPRNs.size();
    if (usedCount >= 10) {
        pdop = 1.5;
        hdop = 0.9;
        vdop = 1.2;
    } else if (usedCount >= 8) {
        pdop = 1.8;
        hdop = 1.1;
        vdop = 1.5;
    } else if (usedCount >= 6) {
        pdop = 2.2;
        hdop = 1.3;
        vdop = 1.8;
    }
    
    // Add DOP values
    char dopStr[30];
    sprintf(dopStr, "%.1f,%.1f,%.1f", pdop, hdop, vdop);
    gsa += "," + String(dopStr);
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 1; i < gsa.length(); i++) {
        checksum ^= gsa[i];
    }
    
    // Add checksum
    gsa += "*";
    if (checksum < 16) gsa += "0";
    gsa += String(checksum, HEX);
    gsa.toUpperCase();
    
    // Add proper line ending for NMEA messages
    gsa += "\r\n";
    
    return gsa;
}

int SatelliteDataManager::getTotalSatellites() const {
    return satellites.size();
}

int SatelliteDataManager::getVisibleSatellites() const {
    int count = 0;
    for (const auto& pair : satellites) {
        if (pair.second.snr > 0) {
            count++;
        }
    }
    return count;
}

// Get count of used satellites
int SatelliteDataManager::getUsedSatellitesCount() const {
    int count = 0;
    for (const auto& pair : satellites) {
        if (pair.second.used) {
            count++;
        }
    }
    return count;
}

// Get list of used satellite PRNs
std::vector<int> SatelliteDataManager::getUsedSatellites() {
    // Return a list of PRNs for satellites marked as used for position fix
    
    std::vector<int> result;
    
    // First add all satellites that are marked as used
    for (const auto& pair : satellites) {
        if (pair.second.used) {
            // Add the original PRN, not the uniqueId we use in the map
            result.push_back(pair.second.prn);
        }
    }
    
    // If we don't have any used satellites from the database,
    // return the usedPRNs vector we've been maintaining
    if (result.empty()) {
        return usedPRNs;
    }
    
    return result;
}

bool SatelliteDataManager::shouldTransmit() {
    unsigned long now = millis();
    
    // Don't transmit more frequently than once per second no matter what
    if (now - lastTransmitTime < 1000) {
        return false;
    }
    
    // Transmit in these conditions:
    // 1. It's been more than UPDATE_INTERVAL since the last transmission
    // 2. We have a full satellite update and it's been at least 2 seconds
    //    since the last transmission
    // 3. We have very few satellites (< 3) and it's been 3 seconds since last
    //    transmission (try to get more data showing)
    
    // Regular update interval
    if (now - lastTransmitTime > UPDATE_INTERVAL) {
        return true;
    }
    
    // We have a full satellite update after the last transmission
    if (lastFullUpdate > lastTransmitTime && now - lastTransmitTime > 2000) {
        return true;
    }
    
    // We have very few satellites, try more frequent updates
    if (getVisibleSatellites() < 3 && now - lastTransmitTime > 3000) {
        return true;
    }
    
    // Adaptive update frequency based on satellite count changes
    static int lastSatCount = -1;
    int currentSatCount = getVisibleSatellites();
    
    // If satellite count has changed significantly, update sooner
    if (lastSatCount >= 0 && 
        abs(currentSatCount - lastSatCount) > 2 && 
        now - lastTransmitTime > 2000) {
        lastSatCount = currentSatCount;
        return true;
    }
    
    // Update the last satellite count
    lastSatCount = currentSatCount;
    
    return false;
}

void SatelliteDataManager::markTransmitted() {
    lastTransmitTime = millis();
}

void SatelliteDataManager::cleanup() {
    unsigned long now = millis();
    std::vector<int> expiredPRNs;
    
    // Identify expired satellites - only expire if well past timeout
    // This makes the system more resilient to temporary signal loss
    for (const auto& pair : satellites) {
        // Only expire satellites if they've been gone for more than 2x the timeout
        if (now - pair.second.lastSeen > (SAT_TIMEOUT * 2)) {
            expiredPRNs.push_back(pair.first);
        }
    }
    
    // If we have very few satellites, don't expire any
    if (satellites.size() <= 4 || expiredPRNs.size() > satellites.size() / 2) {
        // Don't expire any satellites if we have a small number
        // or if we'd expire more than half our database
        return;
    }
    
    // Remove expired satellites
    for (int prn : expiredPRNs) {
        satellites.erase(prn);
    }
    
    // Remove expired PRNs from used list
    auto it = std::remove_if(usedPRNs.begin(), usedPRNs.end(), 
        [this](int prn) {
            return satellites.find(prn) == satellites.end();
        });
    
    if (it != usedPRNs.end()) {
        usedPRNs.erase(it, usedPRNs.end());
    }
}

// This method prioritizes satellites that should be kept in the display
// It's used to prevent important satellites from being overwritten
bool SatelliteDataManager::isPrioritizedForDisplay(int prn) const {
    // First check if this PRN appears in our satellites database
    // (note: this checks the originalPRN, not the uniqueId)
    for (const auto& pair : satellites) {
        if (pair.second.prn == prn) {
            // If this satellite is used for position fix, it's high priority
            if (pair.second.used) {
                return true;
            }
            
            // If it has good signal strength, also higher priority
            if (pair.second.snr >= 20) {
                return true;
            }
        }
    }
    
    // If this PRN is in our usedPRNs list but not in the database yet,
    // it's still prioritized
    if (std::find(usedPRNs.begin(), usedPRNs.end(), prn) != usedPRNs.end()) {
        return true;
    }
    
    // Not a high priority satellite
    return false;
} 