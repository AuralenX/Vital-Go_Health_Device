// sensor_init.ino
#include "bitmaps.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include "heartRate.h"
#include "spo2_algorithm.h"

// Use extern for variables defined in sensor_globals.ino
extern Adafruit_BMP280 bmp;
extern Adafruit_MPU6050 mpu;
extern MAX30105 particleSensor;
extern bool bmp280Detected;
extern bool mpu6050Detected;
extern bool max30102Detected;
extern float lastAccelX, lastAccelY, lastAccelZ;

// Function declarations
void printSensorStatus();
bool hasMinimumSensors();
String getSensorStatus();
void calibrateMPU6050();


// Initialize all sensors
// Initialize all sensors - CORRECTED VERSION
void initSensors() {
  Serial.println("=== INITIALIZING SENSORS ===");
  
  // Initialize I2C at standard speed
  Wire.begin();
  Wire.setClock(100000);  // Start with 100kHz for all devices
  delay(100);
  
  // Scan I2C bus first to see what's connected
  Serial.println("Scanning I2C bus...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify common devices
      if (address == 0x3C || address == 0x30) Serial.print(" (OLED DISPLAY)");
      if (address == 0x57 || address == 0x5A) Serial.print(" (MAX30102)");
      if (address == 0x76 || address == 0x77) Serial.print(" (BMP280)");
      if (address == 0x68 || address == 0x69) Serial.print(" (MPU6050)");
      Serial.println();
    }
  }
  Serial.println("Scan complete.");
  delay(200);
  
  // 1. Initialize MAX30102 FIRST (most sensitive to I2C timing)
  Serial.print("\nMAX30102: ");
  max30102Detected = false;
  
  // Try both common addresses
  if (!particleSensor.begin(Wire, 100000, 0x57)) {
    Serial.print("0x57 failed, trying 0x5A... ");
    if (!particleSensor.begin(Wire, 100000, 0x5A)) {
      Serial.println("FAILED at both addresses");
      max30102Detected = false;
    } else {
      Serial.println("Found at 0x5A!");
      max30102Detected = true;
    }
  } else {
    Serial.println("Found at 0x57!");
    max30102Detected = true;
  }
  
  if (max30102Detected) {
    // Configure MAX30102 with proper settings
    setupMAX30102();
    delay(200);
    
    // Verify with a reading
    long irTest = particleSensor.getIR();
    Serial.print("  Initial IR reading: ");
    Serial.println(irTest);
    
    if (irTest < 1000) {
      Serial.println("  WARNING: Very low IR reading - check wiring!");
    }
  }
  
  // 2. Initialize BMP280
  Serial.print("\nBMP280: ");
  bmp280Detected = false;
  
  // Try primary address 0x76
  if (bmp.begin(0x76)) {
    bmp280Detected = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("OK at 0x76");
  } else {
    // Try alternative address 0x77
    Serial.print("0x76 failed, trying 0x77... ");
    if (bmp.begin(0x77)) {
      bmp280Detected = true;
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                      Adafruit_BMP280::SAMPLING_X2,
                      Adafruit_BMP280::SAMPLING_X16,
                      Adafruit_BMP280::FILTER_X16,
                      Adafruit_BMP280::STANDBY_MS_500);
      Serial.println("OK at 0x77");
    } else {
      Serial.println("FAILED at both addresses");
    }
  }
  
  // 3. Initialize MPU6050
  Serial.print("\nMPU6050: ");
  mpu6050Detected = false;
  
  if (mpu.begin(0x68)) {
    mpu6050Detected = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("OK at 0x68");
  } else {
    Serial.print("0x68 failed, trying 0x69... ");
    if (mpu.begin(0x69)) {
      mpu6050Detected = true;
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_250_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.println("OK at 0x69");
    } else {
      Serial.println("FAILED at both addresses");
    }
  }
  
  // Keep I2C at 100kHz for stability with multiple sensors
  Wire.setClock(100000);
  Serial.println("\nI2C speed set to 100kHz for multi-sensor stability");
  
  // Final status
  delay(500);
  printSensorStatus();
  
  Serial.println("=== SENSOR INITIALIZATION COMPLETE ===\n");
}

// Print sensor status
void printSensorStatus() {
  Serial.println("\n=== SENSOR STATUS ===");
  Serial.println("BMP280   : " + String(bmp280Detected ? "DETECTED" : "NOT DETECTED"));
  Serial.println("MPU6050  : " + String(mpu6050Detected ? "DETECTED" : "NOT DETECTED"));
  Serial.println("MAX30102 : " + String(max30102Detected ? "DETECTED" : "NOT DETECTED"));
  Serial.println("====================\n");
}

// Calibrate MPU6050
void calibrateMPU6050() {
  if (!mpu6050Detected) {
    Serial.println("MPU6050 not detected!");
    return;
  }
  
  Serial.println("Calibrating MPU6050... Keep sensor still!");
  delay(2000);
  
  sensors_event_t a, g, temp;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  
  for (int i = 0; i < 100; i++) {
    mpu.getEvent(&a, &g, &temp);
    accelXSum += a.acceleration.x;
    accelYSum += a.acceleration.y;
    accelZSum += a.acceleration.z;
    if (i % 20 == 0) Serial.print(".");
    delay(10);
  }
  
  lastAccelX = accelXSum / 100;
  lastAccelY = accelYSum / 100;
  lastAccelZ = accelZSum / 100;
  
  Serial.println("\nCalibration complete!");
  Serial.print("Offsets - X: ");
  Serial.print(lastAccelX);
  Serial.print(" Y: ");
  Serial.print(lastAccelY);
  Serial.print(" Z: ");
  Serial.println(lastAccelZ);
}

// Check if minimum sensors are available
bool hasMinimumSensors() {
  return (max30102Detected || bmp280Detected || mpu6050Detected);
}

// Get sensor status string
String getSensorStatus() {
  if (!hasMinimumSensors()) return "NO SENSORS";
  
  String status = "";
  if (max30102Detected) status += "HR ";
  if (bmp280Detected) status += "TEMP ";
  if (mpu6050Detected) status += "MOTION ";
  
  return status;
}