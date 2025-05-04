#include "ButtonHandler.h"
#include <Arduino.h>

#define BUTTON_PIN 39
#define DEBOUNCE_MS 20
#define LONG_PRESS_MS 1000
#define DOUBLE_PRESS_MS 300

ButtonHandler::ButtonHandler() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    lastState = HIGH;
    lastDebounceTime = 0;
    lastPressTime = 0;
    lastReleaseTime = 0;
    pressCount = 0;
    shortPressDetected = false;
    longPressDetected = false;
    doublePressDetected = false;
    pressed = false;
}

void ButtonHandler::update() {
    int reading = digitalRead(BUTTON_PIN);
    unsigned long now = millis();
    if (reading != lastState) {
        lastDebounceTime = now;
    }
    if ((now - lastDebounceTime) > DEBOUNCE_MS) {
        if (reading == LOW && !pressed) {
            pressed = true;
            lastPressTime = now;
        }
        if (reading == HIGH && pressed) {
            pressed = false;
            unsigned long pressDuration = now - lastPressTime;
            if (pressDuration >= LONG_PRESS_MS) {
                longPressDetected = true;
            } else {
                if (now - lastReleaseTime < DOUBLE_PRESS_MS) {
                    doublePressDetected = true;
                    pressCount = 0;
                } else {
                    pressCount = 1;
                }
                shortPressDetected = true;
            }
            lastReleaseTime = now;
        }
    }
    lastState = reading;
}

bool ButtonHandler::wasShortPress() {
    update();
    if (shortPressDetected && !doublePressDetected) {
        shortPressDetected = false;
        return true;
    }
    return false;
}

bool ButtonHandler::wasLongPress() {
    update();
    if (longPressDetected) {
        longPressDetected = false;
        return true;
    }
    return false;
}

bool ButtonHandler::wasDoublePress() {
    update();
    if (doublePressDetected) {
        doublePressDetected = false;
        return true;
    }
    return false;
} 