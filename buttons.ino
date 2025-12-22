// Button handling functions
#include "bitmaps.h"

// Button state variables
bool buttonActive = false;
bool longPressActive = false;
unsigned long buttonPressTime = 0;

void handleButton() {
  int buttonState = digitalRead(BUTTON_PIN);
  unsigned long currentTime = millis();
  
  if (buttonState == LOW) {
    if (!buttonActive) {
      buttonActive = true;
      buttonPressTime = currentTime;
      longPressActive = false;
    } else if (!longPressActive) {
      if (!powerState && currentTime - buttonPressTime > VERY_LONG_PRESS_TIME) {
        longPressActive = true;
        Serial.println("Power on detected!");
        powerOn();
      }
      else if (powerState && currentTime - buttonPressTime > LONG_PRESS_TIME) {
        longPressActive = true;
        Serial.println("Long press - activating SOS!");
        activateSOS();
      }
    }
  } else if (buttonActive) {
    unsigned long pressDuration = currentTime - buttonPressTime;
    
    if (powerState) {
      if (pressDuration < SHORT_PRESS_TIME) {
        Serial.println("Short press - next page");
        nextPage();
        pageTimer = currentTime;
        lastActivityTime = currentTime;
      } else if (pressDuration > VERY_LONG_PRESS_TIME) {
        Serial.println("Very long press - powering off");
        powerOff();
      }
    }
    
    buttonActive = false;
    longPressActive = false;
  }
}