// All display drawing functions
#include "bitmaps.h"

// Animation state variables
int heartbeatFrame = 0;
int respFrame = 0;
int walkFrame = 0;
int scrollPos = 0;
unsigned long lastHeartbeatUpdate = 0;
unsigned long lastResUpdate = 0;
unsigned long lastWalkUpdate = 0;
unsigned long lastScroll = 0;

void drawLogoPage() {
  display.clearDisplay();
  unsigned long elapsedTime = millis() - logoStartTime;
  int fadeProgress = map(elapsedTime, 0, 3000, 0, 128);
  if (fadeProgress > 128) fadeProgress = 128;
  
  display.drawBitmap(0, 0, epd_bitmap_logo, fadeProgress, 64, SSD1306_WHITE);
 
  unsigned long logoElapsed = millis() - logoStartTime;
  int progress = map(logoElapsed, 0, LOGO_DURATION * 3 / 4, 0, 120);
  if (progress > 120) progress = 120;

  display.drawRect(4, 60, 120, 4, SSD1306_WHITE);
  display.fillRect(4, 60, progress, 4, SSD1306_WHITE);
  
  if (logoElapsed > (LOGO_DURATION * 3 / 4)) {
    drawSensorStatus();
  }
}
// Vitals screen 1
void drawVitalsPage1() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(20, 2);
  display.println("VITAL SIGNS 1/2");

  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 18);
  display.print("Heart Rate:");
  display.setCursor(90, 18);
  display.print(heartRate);
  display.setCursor(110, 18);
  display.print("BPM");

  display.drawBitmap(50, 32, heartbeat_frames[heartbeatFrame], 16, 16, SSD1306_WHITE);

  display.setCursor(10, 36);
  display.print("SpO2:");
  display.setCursor(90, 36);
  display.print(bloodOxygen);
  display.setCursor(110, 36);
  display.print("%");

  display.setCursor(10, 52);
  display.print("Temperature:");
  display.setCursor(90, 52);
  display.print(bodyTemperature, 1);
  display.setCursor(115, 52);
  display.print("C");
}
// Vitals screen 2
void drawVitalsPage2() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(20, 2);
  display.println("VITAL SIGNS 2/2");

  display.setTextColor(SSD1306_WHITE);
  display.setCursor(2, 18);
  display.print("Blood P.:");
  display.setCursor(60, 18);
  display.print(bloodPressureSystolic);
  display.setCursor(80, 18);
  display.print("/");
  display.setCursor(85, 18);
  display.print(bloodPressureDiastolic);
  display.setCursor(100, 18);
  display.print("mmHg");

  display.setCursor(2, 36);
  display.print("R. Rate:");
  display.setCursor(90, 36);
  display.print(respiratoryRate);
  display.setCursor(110, 36);
  display.print("BPM");

  // Respiratory animation
  if (millis() - lastResUpdate > 800) {
    respFrame = (respFrame + 1) % 3;
    lastResUpdate = millis();
  }
  int lineLength = 10 + (respFrame * 10);
  display.drawRect(50, 38, lineLength, 4, SSD1306_WHITE);

  display.setCursor(2, 50);
  display.print("Sleep Quality:");
  display.setCursor(100, 50);
  display.print(sleepQuality);
  display.setCursor(115, 50);
  display.print("%");
}

void drawActivityPage() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(40, 2);
  display.println("ACTIVITY");

  display.setTextColor(SSD1306_WHITE);

  // Walking animation
  if (millis() - lastWalkUpdate > 500) {
    walkFrame = (walkFrame + 1) % 2;
    lastWalkUpdate = millis();
  }

  int walkX = 30 + walkFrame * 3;
  display.fillCircle(walkX, 30, 5, SSD1306_WHITE);
  display.drawLine(walkX, 35, walkX, 45, SSD1306_WHITE);
  display.drawLine(walkX, 38, 25 + walkFrame * 2, 35, SSD1306_WHITE);
  display.drawLine(walkX, 38, 35 - walkFrame * 2, 35, SSD1306_WHITE);
  display.drawLine(walkX, 45, 25 + walkFrame, 52, SSD1306_WHITE);
  display.drawLine(walkX, 45, 35 - walkFrame, 52, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(60, 25);
  display.print(stepsToday);

  display.setTextSize(1);
  display.setCursor(60, 50);
  display.print("today:");

  int stepProgress = map(stepsToday, 0, 10000, 0, 100);
  if (stepProgress > 100) stepProgress = 100;

  display.drawRect(10, 60, 98, 4, SSD1306_WHITE);
  display.fillRect(10, 60, stepProgress, 4, SSD1306_WHITE);

  display.setCursor(95, 50);
  display.print(stepProgress);
  display.print("%");
}
// System Screen
void drawSystemPage() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(30, 2);
  display.println("SYSTEM INFO");

  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 20);
  display.print("Battery:");

  display.drawRect(60, 18, 40, 12, SSD1306_WHITE);
  display.fillRect(100, 21, 2, 6, SSD1306_WHITE);
  int batteryWidth = map(batteryLevel, 0, 95, 0, 38);
  display.fillRect(60, 19, batteryWidth, 10, SSD1306_WHITE);

  display.setCursor(105, 20);
  display.print(batteryLevel);
  display.print("%");

  display.setCursor(10, 40);
  display.print("Signal:");

  for (int i = 0; i < 5; i++) {
    if (i < signalStrength) {
      display.fillRect(60 + i * 6, 40 - (i * 2), 5, 10 + (i * 2), SSD1306_WHITE);
    } else {
      display.drawRect(60 + i * 6, 40 - (i * 2), 5, 10 + (i * 2), SSD1306_WHITE);
    }
  }

  display.setCursor(105, 40);
  display.print(signalStrength);
  display.print("/5");
  
  display.setCursor(2, 55);
  display.print("Status: ");
  display.print(sosActive ? "EMERGENCY" : "NORMAL");

  display.setCursor(100, 55);
  display.print("Pg ");
  display.print(currentPage + 1);
}
// About screen
void drawAboutPage() {
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(45, 2);
  display.println("ABOUT");
  
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(2, 18);
  display.print("Version: 1.0.0");
  display.setCursor(2, 28);
  display.print("Firmware: v1.1");
  
  if (millis() - lastScroll > 300) {
    scrollPos = (scrollPos + 1) % 43;
    lastScroll = millis();
  }
  
  display.setCursor(2, 43);
  display.print("Web:");
  
  const char* url = "https://vital-go.com/patient-monitoring";
  display.setCursor(30, 43);
  for (int i = 0; i < 15; i++) {
    int pos = (scrollPos + i) % 43;
    display.print(url[pos]);
  }
  
  display.setCursor(10, 55);
  display.print("(C) 2025 Vital-Go");
}

void drawQRCodePage() {
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(25, 2);
  display.println("SCAN QR CODE");
  
  int qrX = (128 - 50) / 2;  
  int qrY = (64 - 50) / 2 + 8;
  
  display.drawRect(qrX - 2, qrY - 2, 54, 54, SSD1306_WHITE);
  display.drawBitmap(qrX, qrY, epd_bitmap_vitalgo_qrcode, 50, 50, SSD1306_WHITE);
  
  display.setCursor(110, 55);
  display.print("QR");
}
// SOS screen
void drawSOSScreen() {
  static unsigned long lastFlash = 0;
  static bool flashState = true;

  if (millis() - lastFlash > 250) {
    flashState = !flashState;
    lastFlash = millis();
  }

  if (flashState) {
    display.fillScreen(SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }

  display.setTextSize(1);
  display.setCursor(40, 5);
  display.println("EMERGENCY!");
  display.setCursor(10, 30);
  display.println("SOS Alert Activated");
  display.setCursor(15, 40);
  display.println("Help is on the way");

  unsigned long sosDuration = (millis() - sosStartTime) / 1000;
  display.setCursor(40, 55);
  display.print("Time: ");
  display.print(30 - sosDuration);
  display.print("s");

  if (sosDuration > 30) {
    deactivateSOS();
  }
}
// Sensor status
void drawSensorStatus() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(28, 18);
  display.print("Sensor Status:");
  
  String status = getSensorStatus();
  
  int textWidth = status.length() * 6;
  int xPos = 64 - (textWidth / 2);
  display.setCursor(xPos, 35);
  
  if (status == "NO SENSORS") {
    display.print("NO SENSORS");
  } else {
    display.print(status);
  }
  
  display.setCursor(20, 28);
  if (max30102Detected) {
    display.print("Heart monitor");
    display.drawCircle(5, 35, 2, SSD1306_WHITE); 
  }
  
  if (bmp280Detected) {
    display.setCursor(20, 41);
    display.print("Temperature");
    display.drawCircle(5, 45, 2, SSD1306_WHITE); 
  }
  
  if (mpu6050Detected) {
    display.setCursor(20, 55);
    display.print("Movement");
    display.drawCircle(5, 55, 2, SSD1306_WHITE); 
  }
}