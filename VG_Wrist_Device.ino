// VG_Wrist_Device.ino - MAIN FILE
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "bitmaps.h"

// Global OLED object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ========== EXTERN VARIABLES (declared in sensor_globals.ino) ==========
extern bool powerState;
extern bool sosActive;
extern bool showingLogo;
extern PageState currentPage;
extern unsigned long logoStartTime;
extern unsigned long pageTimer;
extern unsigned long lastActivityTime;
extern unsigned long sosStartTime;
extern int heartbeatFrame;
extern unsigned long lastHeartbeatUpdate;

// ========== FUNCTION PROTOTYPES ==========
// Power functions
void powerOn();
void powerOff();
void activateSOS();
void deactivateSOS();
void nextPage();
void drawPowerOnAnimation();
void drawPowerOffAnimation();
void drawPowerOffScreen();

// Display functions
void drawLogoPage();
void drawVitalsPage1();
void drawVitalsPage2();
void drawActivityPage();
void drawSystemPage();
void drawAboutPage();
void drawQRCodePage();
void drawSOSScreen();

// Button functions
void handleButton();

// Sensor functions
void initSensors();
void updateSensorData();
void checkVitalAlerts();
String getSensorStatus();
void calibrateMPU6050();
void handleSerialCommands();

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  
  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  
  // Initialize button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  Serial.println("==========================================");
  Serial.println("      VITAL-GO WEARABLE HEALTH MONITOR   ");
  Serial.println("==========================================");
  
  // Initialize all sensors
  initSensors();
  
  // Initialize system
  powerOn();
  
  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("Place finger on sensor for heart rate readings");
  Serial.println("Move device to count steps");
  Serial.println("Type 'help' for serial commands");
  Serial.println("==========================================\n");
}

// ========== LOOP ==========
void loop() {
  unsigned long currentTime = millis();
  
  // Handle button input
  handleButton();
  
  // Check power state
  if (!powerState) {
    delay(100);
    return;
  }
  
  // Auto power-off due to inactivity
  if (!sosActive && currentTime - lastActivityTime > POWER_OFF_TIMEOUT) {
    Serial.println("Auto power-off due to inactivity");
    powerOff();
    return;
  }
  
  // Check if still showing logo after power on
  if (showingLogo) {
    if (currentTime - logoStartTime > LOGO_DURATION) {
      showingLogo = false;
      pageTimer = currentTime;
      lastActivityTime = currentTime;
    } else {
      drawLogoPage();
      display.display();
      delay(50);
      return;
    }
  }
  
  // Auto-advance pages
  if (!sosActive && currentTime - pageTimer > PAGE_DURATION) {
    nextPage();
    pageTimer = currentTime;
  }
  
  // Update heartbeat animation
  if (currentTime - lastHeartbeatUpdate > HEARTBEAT_INTERVAL) {
    heartbeatFrame = (heartbeatFrame + 1) % 3;
    lastHeartbeatUpdate = currentTime;
  }
  
  // Update display
  display.clearDisplay();
  
  if (sosActive) {
    drawSOSScreen();
  } else {
    switch (currentPage) {
      case PAGE_VITALS_1:
        drawVitalsPage1();
        break;
      case PAGE_VITALS_2:
        drawVitalsPage2();
        break;
      case PAGE_ACTIVITY:
        drawActivityPage();
        break;
      case PAGE_SYSTEM:
        drawSystemPage();
        break;
      case PAGE_ABOUT:
        drawAboutPage();
        break;
      case PAGE_QR_CODE:
        drawQRCodePage();
        break;
    }
  }
  
  display.display();
  
  // Update sensor data
  updateSensorData();
  
  // Handle serial commands
  handleSerialCommands();
  
  delay(50); 
}