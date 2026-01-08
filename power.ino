#include "bitmaps.h"
#include <Adafruit_SSD1306.h>

extern Adafruit_SSD1306 display;

// Use extern for all variables
extern bool powerState;
extern bool sosActive;
extern bool showingLogo;
extern PageState currentPage;
extern unsigned long logoStartTime;
extern unsigned long pageTimer;
extern unsigned long sosStartTime;
extern unsigned long lastActivityTime;

extern int stepCount;
extern int stepsToday;
extern int heartRate;
extern int bloodOxygen;
extern bool mpu6050Detected;

// Function prototypes
void calibrateMPU6050();
void drawPowerOnAnimation();
void drawPowerOffAnimation();
void drawPowerOffScreen();

void powerOn() {
  powerState = true;
  sosActive = false;
  showingLogo = true;
  logoStartTime = millis() + 1000;
  currentPage = PAGE_VITALS_1;
  pageTimer = millis();
  lastActivityTime = millis();

  stepCount = 0;
  stepsToday = 0;
  heartRate = 0;
  bloodOxygen = 0;
  
  // Calibrate motion sensor
  if (mpu6050Detected) {
    calibrateMPU6050();
  }
  
  display.clearDisplay();
  drawPowerOnAnimation();
  display.display();
  delay(1000);
  
  Serial.println("=== SYSTEM POWERED ON ===");
  Serial.println("Page: 1");
}

void powerOff() {
  Serial.println("=== SYSTEM POWERING OFF ===");
  
  display.clearDisplay();
  drawPowerOffAnimation();
  display.display();
  delay(1000);
  
  display.clearDisplay();
  drawPowerOffScreen();
  display.display();
  
  powerState = false;
  sosActive = false;
  showingLogo = false;
  currentPage = PAGE_VITALS_1;
  
  Serial.println("System off. Press button for 5s to power on.");
}

void drawPowerOnAnimation() {
  for (int i = 0; i <= 100; i += 5) {
    display.clearDisplay();
    
    int centerX = 64;
    int centerY = 32;
    int radius = 15;
    
    display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
    
    int fillAngle = map(i, 0, 100, 0, 360);
    for (int angle = 0; angle < fillAngle; angle += 10) {
      float rad = angle * PI / 180.0;
      int x = centerX + (radius - 2) * cos(rad);
      int y = centerY + (radius - 2) * sin(rad);
      display.drawPixel(x, y, SSD1306_WHITE);
    }
    
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 50);
    display.print("POWERING ON");
    
    // Draw dots animation
    static int dots = 0;
    dots = (dots + 1) % 4;
    for (int d = 0; d < dots; d++) {
      display.print(".");
    }
    display.display();
    delay(50);
  }
}

void drawPowerOffAnimation() {
  for (int i = 100; i >= 0; i -= 5) {
    display.clearDisplay();
    
    int centerX = 64;
    int centerY = 32;
    int radius = 15;
    
    display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
    
    int fillAngle = map(i, 0, 100, 0, 360);
    for (int angle = 0; angle < fillAngle; angle += 10) {
      float rad = angle * PI / 180.0;
      int x = centerX + (radius - 2) * cos(rad);
      int y = centerY + (radius - 2) * sin(rad);
      display.drawPixel(x, y, SSD1306_WHITE);
    }
    
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(30, 50);
    display.print("POWERING OFF");
    
    display.display();
    delay(30);
  }
}

void drawPowerOffScreen() {
  display.clearDisplay();
  
  int centerX = 64;
  int centerY = 25;
  int radius = 15;
  
  display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
  display.drawLine(centerX, centerY - radius + 5, centerX, centerY - 5, SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(30, 45);
  display.print("SYSTEM OFF");
  display.setCursor(5, 55);
  display.print("Hold 5s to power on");
}

void activateSOS() {
  sosActive = true;
  sosStartTime = millis();
  lastActivityTime = millis();
  
  Serial.println("=== EMERGENCY SOS ACTIVATED ===");
  Serial.println("Sending alerts to caregivers...");
  drawSOSScreen();
  
  // Log which sensors contributed to the alert
  Serial.print("Active sensors: ");
  if (max30102Detected) Serial.print("HR/SpO2 ");
  if (bmp280Detected) Serial.print("TEMP ");
  if (mpu6050Detected) Serial.print("MOTION ");
  Serial.println();
}

void deactivateSOS() {
  sosActive = false;
  lastActivityTime = millis();
  Serial.println("SOS deactivated");
}

void nextPage() {
  currentPage = (PageState)((currentPage + 1) % PAGE_COUNT);
  lastActivityTime = millis();
  Serial.print("Page: ");
  Serial.println(currentPage);
}