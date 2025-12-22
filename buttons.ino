// Button handling functions
#include "bitmaps.h"

bool buttonActive = false;
bool longPressActive = false;
unsigned long buttonPressTime = 0;

// For double press detection
#define DOUBLE_PRESS_TIME 300        
#define SHORT_PRESS_MAX 1000         
#define LONG_PRESS_MIN 2000          
#define POWER_ON_PRESS_TIME 3000     
#define DEBOUNCE_TIME 200            

unsigned long lastReleaseTime = 0;
bool waitingForSecondPress = false;
int pressCount = 0;

void handleButton() {
  static unsigned long lastPressTime = 0;
  static int pressCount = 0;
  static bool longPressHandled = false;
  
  int buttonState = digitalRead(BUTTON_PIN);
  unsigned long currentTime = millis();
  
  if (buttonState == LOW) {
    if (!buttonActive) {
      buttonActive = true;
      buttonPressTime = currentTime;
      longPressHandled = false;
    } else {
      // Check for power-on when device is OFF
      if (!powerState && (currentTime - buttonPressTime > POWER_ON_PRESS_TIME)) {
        if (!longPressHandled) {
          longPressHandled = true;
          Serial.println("Power on detected!");
          powerOn();
          return; // Exit early since we handled power on
        }
      }
      // Check for long press (SOS) when device is ON
      else if (powerState && !longPressHandled && (currentTime - buttonPressTime > LONG_PRESS_MIN)) {
        longPressHandled = true;
        Serial.println("Long press - activating SOS!");
        activateSOS();
      }
    }
  } else if (buttonActive) {
    unsigned long pressDuration = currentTime - buttonPressTime;
    buttonActive = false;
    
    // Only process short press logic if device is ON and not a long press
    if (powerState && pressDuration < LONG_PRESS_MIN && !longPressHandled) {
      // Short press detected
      if (currentTime - lastPressTime < DOUBLE_PRESS_TIME) {
        pressCount++;
      } else {
        pressCount = 1;
      }
      
      lastPressTime = currentTime;
      
      if (pressCount == 2) {
        // Double press detected
        Serial.println("Double press - powering off");
        powerOff();
        pressCount = 0;
      } else {
       
      }
    }
    // Reset long press flag on release (unless it was power on)
    if (powerState || pressDuration < POWER_ON_PRESS_TIME) {
      longPressHandled = false;
    }
  }
  
  // Handle single press after timeout (only when device is ON)
  if (powerState && pressCount == 1 && (currentTime - lastPressTime > DOUBLE_PRESS_TIME)) {
    Serial.println("Single press - next page");
    nextPage();
    pageTimer = currentTime;
    lastActivityTime = currentTime;
    pressCount = 0;
  }

}