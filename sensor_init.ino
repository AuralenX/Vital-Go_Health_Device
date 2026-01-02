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
void initSensors() {
  Serial.println("=== INITIALIZING SENSORS ===");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000);
  delay(100);
  
  // 1. Initialize BMP280
  Serial.print("BMP280 at 0x");
  Serial.print(0x76, HEX);
  Serial.print(": ");
  if (bmp.begin(0x76)) {
    bmp280Detected = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("OK");
    Serial.print("  Temp: ");
    Serial.print(bmp.readTemperature());
    Serial.println("°C");
  } else {
    Serial.println("FAILED");
  }
  delay(200);
  
  // 2. Initialize MPU6050
  Serial.print("MPU6050 at 0x");
  Serial.print(MPU6050_ADDRESS, HEX);
  Serial.print(": ");
  Wire.beginTransmission(MPU6050_ADDRESS);
  byte mpuError = Wire.endTransmission();
  
  if (mpuError == 0 && mpu.begin(MPU6050_ADDRESS)) {
    mpu6050Detected = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("OK");
  } else {
    Serial.print("FAILED (error: ");
    Serial.print(mpuError);
    Serial.println(")");
  }
  delay(200);
  
  // 3. Initialize MAX30102
  Serial.print("MAX30102 at 0x");
  Serial.print(MAX30102_ADDRESS, HEX);
  Serial.print(": ");
  bool maxFound = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    if (particleSensor.begin(Wire, I2C_SPEED_STANDARD, MAX30102_ADDRESS)) {
      maxFound = true;
      break;
    }
    delay(100);
  }
  
  if (maxFound) {
    max30102Detected = true;
    particleSensor.setup(0x1F, 4, 2, 100, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("OK");
    
    // Test IR sensor
    long irTest = particleSensor.getIR();
    Serial.print("  IR test: ");
    Serial.println(irTest);
  } else {
    Serial.println("FAILED");
    // Try alternative address
    if (particleSensor.begin(Wire, I2C_SPEED_STANDARD, 0x5A)) {
      max30102Detected = true;
      Serial.println("  Found at 0x5A!");
    }
  }
  
  // Finalize
  delay(500);
  printSensorStatus();
  
  if (bmp280Detected || mpu6050Detected || max30102Detected) {
    Wire.setClock(400000);
    Serial.println("I2C speed: 400kHz");
  }
  
  Serial.println("===========================\n");
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