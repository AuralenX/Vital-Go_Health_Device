// Main Arduino sketch
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "bitmaps.h"

// Global OLED object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ========== POWER & SYSTEM VARIABLES ==========
extern bool powerState;
extern bool sosActive;
extern bool showingLogo;
extern PageState currentPage;
extern unsigned long logoStartTime;
extern unsigned long pageTimer;
extern unsigned long lastActivityTime;
extern unsigned long sosStartTime;

// ========== SENSOR DETECTION VARIABLES ==========
extern bool bmp280Detected;
extern bool mpu6050Detected;
extern bool max30102Detected;

// ========== VITAL SIGNS DATA ==========
extern int heartRate;
extern int bloodOxygen;
extern float bodyTemperature;
extern float ambientTemperature;
extern int bloodPressureSystolic;
extern int bloodPressureDiastolic;
extern int respiratoryRate;
extern int sleepQuality;
extern int stepsToday;
extern int batteryLevel;
extern int signalStrength;

// ========== MOTION SENSOR VARIABLES ==========
extern int stepCount;
extern float lastAccelX, lastAccelY, lastAccelZ;
extern unsigned long lastStepTime;

// ========== HEART RATE SENSOR VARIABLES ==========
extern long irValue;
extern float beatsPerMinute;
extern int beatAvg;
extern byte rateSpot;
extern long lastBeat;

// ========== TIMING VARIABLES ==========
extern unsigned long lastHRRead;
extern unsigned long lastTempRead;
extern unsigned long lastMotionRead;
extern unsigned long lastRespUpdate;
extern unsigned long lastUpdate;

// ========== ANIMATION STATE VARIABLES ==========
extern int heartbeatFrame;
extern int respFrame;
extern int walkFrame;
extern int scrollPos;
extern unsigned long lastHeartbeatUpdate;
extern unsigned long lastResUpdate;
extern unsigned long lastWalkUpdate;
extern unsigned long lastScroll;

// ========== BUTTON STATE VARIABLES ==========
extern bool buttonActive;
extern bool longPressActive;
extern unsigned long buttonPressTime;

// ========== FUNCTION PROTOTYPES ==========
void powerOn();
void powerOff();
void activateSOS();
void deactivateSOS();
void nextPage();
void drawPowerOnAnimation();
void drawPowerOffAnimation();
void drawPowerOffScreen();

// Sensor functions (sensors.ino)
void initSensors();
void updateSensorData();
void checkVitalAlerts();
String getSensorStatus();
void calibrateMPU6050();
void readMotionData();
void readHeartRateData();
void estimateSpO2();
void estimateBloodPressure(float bpm);
void checkFallDetection(float accelMagnitude, float gyroX, float gyroY, float gyroZ);
void estimateRespiratoryRate(float chestMovement);

// Display functions (display.ino)
void drawLogoPage();
void drawVitalsPage1();
void drawVitalsPage2();
void drawActivityPage();
void drawSystemPage();
void drawAboutPage();
void drawQRCodePage();
void drawSOSScreen();

// Button functions (buttons.ino)
void handleButton();

// Sensor data function (replaces old simulated data)
void updateSimulatedData(); 

// ========== SETUP FUNCTION ==========
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
  pageTimer = millis();
  lastActivityTime = millis();
  
  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("Place finger on sensor for heart rate readings");
  Serial.println("Move device to count steps");
  Serial.println("==========================================\n");
}

// ========== LOOP FUNCTION ==========
void loop() {
  unsigned long currentTime = millis();
  
  // Handle button input (power on/off, SOS, page change)
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
  
  // Auto-advance pages every PAGE_DURATION seconds
  if (!sosActive && currentTime - pageTimer > PAGE_DURATION) {
    nextPage();
    pageTimer = currentTime;
  }
  
  // Update heartbeat animation
  if (currentTime - lastHeartbeatUpdate > HEARTBEAT_INTERVAL) {
    heartbeatFrame = (heartbeatFrame + 1) % 3;
    lastHeartbeatUpdate = currentTime;
  }
  
  // Update display with current page
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
  
  // Update REAL sensor data
  updateSensorData();  
  
  // Check for vital sign emergencies
  checkVitalAlerts();
  
  delay(50); 
}