#include "bitmaps.h"
#include <Adafruit_BMP280.h>

// Use extern
extern Adafruit_BMP280 bmp;
extern bool bmp280Detected;
extern float ambientTemperature;
extern float bodyTemperature;
extern unsigned long lastTempRead;

// Read BMP280 data
void readBMP280Data() {
  if (!bmp280Detected) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastTempRead >= TEMP_READ_INTERVAL) {
    ambientTemperature = bmp.readTemperature();
    bodyTemperature = ambientTemperature + TEMPERATURE_OFFSET;
    lastTempRead = currentTime;
    
    Serial.print("[BMP280] Ambient: ");
    Serial.print(ambientTemperature);
    Serial.print("°C, Body: ");
    Serial.print(bodyTemperature);
    Serial.println("°C");
    
    float pressure = bmp.readPressure() / 100.0F;
    Serial.print("[BMP280] Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");
  }
}

// Print BMP280 debug info
void printBMP280Info() {
  if (!bmp280Detected) {
    Serial.println("BMP280: Not detected");
    return;
  }
  
  Serial.println("=== BMP280 INFO ===");
  Serial.print("Temperature: ");
  Serial.print(bmp.readTemperature());
  Serial.println("°C");
  Serial.print("Pressure: ");
  Serial.print(bmp.readPressure() / 100.0F);
  Serial.println(" hPa");
  Serial.print("Altitude: ");
  Serial.print(bmp.readAltitude(1013.25));
  Serial.println(" m");
  Serial.println("===================\n");
}