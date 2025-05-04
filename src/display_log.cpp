#include "display_log.h"
#include <Arduino.h>
#include <vector>
#include "display_manager.h"
#include "main_globals.h"
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;

// Externs for globals used in these functions
extern bool display_initialized;
extern std::vector<String> log_buffer;
extern int currentLogIndex;
extern DisplayMode currentDisplayMode;
extern void logMessage(const String& msg);

void displayLogMessages() {
  if (!display_initialized) return;

  display.clearDisplay();
  setDisplayTitleStyle();

  // Center the title
  String title = "--- Error Log ---";
  int titleWidth = display.getStrWidth(title.c_str());
  display.setCursor((SCREEN_WIDTH - titleWidth) / 2, 0);
  display.println(title);

  setDisplayDefaultStyle();

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
    
    // Split message into words and wrap
    int currentX = 0;
    String currentLine = "";
    
    for (int i = 0; i < message.length(); i++) {
      char c = message[i];
      currentLine += c;
      
      if (display.getStrWidth(currentLine.c_str()) > maxWidth || c == '\n') {
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
  display.setDrawColor(1);
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
  // No action for long press on log display
} 