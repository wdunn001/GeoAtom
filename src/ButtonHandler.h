#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>

class ButtonHandler {
public:
    ButtonHandler();
    void update();
    bool wasShortPress();
    bool wasLongPress();
    bool wasDoublePress();
private:
    int lastState;
    unsigned long lastDebounceTime;
    unsigned long lastPressTime;
    unsigned long lastReleaseTime;
    int pressCount;
    bool shortPressDetected;
    bool longPressDetected;
    bool doublePressDetected;
    bool pressed;
};

#endif // BUTTON_HANDLER_H 