// mpu6050_sensor.ino
#include "bitmaps.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Use extern for variables
extern Adafruit_MPU6050 mpu;
extern bool mpu6050Detected;
extern float lastAccelX, lastAccelY, lastAccelZ;
extern int stepCount;
extern int stepsToday;
extern unsigned long lastStepTime;
extern float respBuffer[20];
extern int respIndex;
extern int respiratoryRate;
extern unsigned long lastRespUpdate;
extern unsigned long lastMotionRead;

// Function declarations
void estimateRespiratoryRate(float chestMovement);
void checkFallDetection(float accelMagnitude, float gyroX, float gyroY, float gyroZ);

// Read MPU6050 data
void readMPU6050Data() {
  if (!mpu6050Detected) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastMotionRead >= MOTION_READ_INTERVAL) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    lastMotionRead = currentTime;
    
    // Calculate acceleration magnitude
    float accelX = a.acceleration.x - lastAccelX;
    float accelY = a.acceleration.y - lastAccelY;
    float accelZ = a.acceleration.z - lastAccelZ;
    float accelMagnitude = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    
    // Step detection
    if (accelMagnitude > ACCEL_THRESHOLD && 
        millis() - lastStepTime > STEP_DEBOUNCE) {
      stepCount++;
      stepsToday = stepCount;
      lastStepTime = millis();
      
      Serial.print("[MPU6050] Step #");
      Serial.println(stepCount);
    }
    
    // Estimate respiratory rate
    estimateRespiratoryRate(accelZ);
    
    // Fall detection
    checkFallDetection(accelMagnitude, g.gyro.x, g.gyro.y, g.gyro.z);
  }
}

// Estimate respiratory rate
void estimateRespiratoryRate(float chestMovement) {
  respBuffer[respIndex] = chestMovement;
  respIndex = (respIndex + 1) % 20;
  
  if (millis() - lastRespUpdate > 5000) {
    int peaks = 0;
    for (int i = 2; i < 18; i++) {
      if (respBuffer[i] > respBuffer[i-1] && 
          respBuffer[i] > respBuffer[i-2] &&
          respBuffer[i] > respBuffer[i+1] && 
          respBuffer[i] > respBuffer[i+2] &&
          respBuffer[i] > 0.2) {
        peaks++;
      }
    }
    
    if (peaks > 0) {
      respiratoryRate = (peaks * 60) / 5;
      respiratoryRate = constrain(respiratoryRate, RESP_RATE_MIN, RESP_RATE_MAX);
      
      Serial.print("[MPU6050] Respiratory rate: ");
      Serial.println(respiratoryRate);
    }
    
    lastRespUpdate = millis();
  }
}

// Check for falls
void checkFallDetection(float accelMagnitude, float gyroX, float gyroY, float gyroZ) {
  static bool fallDetected = false;
  static unsigned long fallTime = 0;
  static unsigned long freeFallStart = 0;
  static bool freeFallDetected = false;
  
  // Free fall detection (sudden weightlessness)
  if (accelMagnitude > FREE_FALL_THRESHOLD_LOW && 
      accelMagnitude < FREE_FALL_THRESHOLD_HIGH) {
    if (!freeFallDetected) {
      freeFallStart = millis();
      freeFallDetected = true;
    }
  } else {
    freeFallDetected = false;
  }
  
  // Impact detection
  if ((accelMagnitude > FALL_IMPACT_THRESHOLD && !fallDetected) ||
      (freeFallDetected && (millis() - freeFallStart) > MIN_FREE_FALL_TIME)) {
    fallDetected = true;
    fallTime = millis();
    Serial.println("[MPU6050] FALL DETECTED!");
  }
  
  // Check for post-fall stillness
  if (fallDetected && millis() - fallTime > 3000) {
    if (accelMagnitude < FALL_STILLNESS_THRESHOLD) {
      Serial.println("[MPU6050] EMERGENCY: No movement after fall!");
      // activateSOS(); // Uncomment if you have this function
    }
    fallDetected = false;
  }
}

// Print MPU6050 raw data (REMOVE the problematic function or fix it)
void printMPU6050Info() {
  if (!mpu6050Detected) {
    Serial.println("MPU6050: Not detected");
    return;
  }
  
  Serial.println("=== MPU6050 INFO ===");
  
  sensors_event_t a, g, temp;
  if (mpu.getEvent(&a, &g, &temp)) {
    Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s²\t");
    Serial.print("Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s²\t");
    Serial.print("Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s²");
    
    Serial.print("Gyro X: "); Serial.print(g.gyro.x); Serial.print(" rad/s\t");
    Serial.print("Y: "); Serial.print(g.gyro.y); Serial.print(" rad/s\t");
    Serial.print("Z: "); Serial.print(g.gyro.z); Serial.println(" rad/s");
    
    Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println("°C");
  }
  
  Serial.print("Steps today: ");
  Serial.println(stepsToday);
  Serial.print("Respiratory rate: ");
  Serial.println(respiratoryRate);
  Serial.println("====================\n");
}