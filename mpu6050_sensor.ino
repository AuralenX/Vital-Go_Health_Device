// mpu6050_sensor.ino - Optimized for wrist wearable
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
extern unsigned long lastMotionRead;

// Wrist-specific variables
float wristCalibrationOffsetX = 0, wristCalibrationOffsetY = 0, wristCalibrationOffsetZ = 0;
bool isCalibrated = false;

// Step detection (wrist-optimized)
float prevStepSignal = 0;
unsigned long lastStepDetectedTime = 0;

// Fall detection variables
unsigned long fallDetectionStart = 0;

// Activity classification
ActivityState currentActivity = ACTIVITY_IDLE;
float lastAccelMagnitude = 0;


// Activity smoothing
float avgAccel = 0;
float avgGyro = 0;

// Sleep detection
unsigned long stillnessCounter = 0;
bool isSleeping = false;

// Activity state to string conversion
String activityToString(ActivityState activity) {
  switch (activity) {
    case ACTIVITY_IDLE: return "IDLE";
    case ACTIVITY_WALKING: return "WALKING";
    case ACTIVITY_RUNNING: return "RUNNING";
    case ACTIVITY_SHAKING: return "SHAKING";
    case ACTIVITY_UNKNOWN: return "UNKNOWN";
    default: return "UNKNOWN";
  }
}

// ========== INITIALIZATION ==========

// Setup MPU6050 for wrist wearable
void setupMPU6050() {
  if (!mpu6050Detected) return;
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("[MPU6050] Configured for wrist wearable");
  Serial.println("  Accel Range: ±4G");
  Serial.println("  Gyro Range: ±500°/s");
  Serial.println("  Filter: 21Hz");
}

// ========== WRIST CALIBRATION ==========

// Calibrate wrist position
void calibrateWristPosition() {
  if (!mpu6050Detected) {
    Serial.println("[MPU6050] Sensor not detected for calibration");
    return;
  }
  
  Serial.println("[MPU6050] Calibrating wrist position...");
  Serial.println("Please keep your wrist still and relaxed for 3 seconds");
  
  delay(2000);
  
  sensors_event_t a, g, temp;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  int samples = 60;
  
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    
    accelXSum += a.acceleration.x;
    accelYSum += a.acceleration.y;
    accelZSum += a.acceleration.z;
    
    gyroXSum += g.gyro.x;
    gyroYSum += g.gyro.y;
    gyroZSum += g.gyro.z;
    
    if (i % 15 == 0) {
      Serial.print(".");
    }
    
    delay(50);
  }
  
  wristCalibrationOffsetX = accelXSum / samples;
  wristCalibrationOffsetY = accelYSum / samples;
  wristCalibrationOffsetZ = accelZSum / samples;
  
  float gyroAvgX = gyroXSum / samples;
  float gyroAvgY = gyroYSum / samples;
  float gyroAvgZ = gyroZSum / samples;
  
  isCalibrated = true;
  
  Serial.println("\n[MPU6050] Wrist calibration complete!");
  Serial.print("Gravity vector - X: ");
  Serial.print(wristCalibrationOffsetX, 2);
  Serial.print(" Y: ");
  Serial.print(wristCalibrationOffsetY, 2);
  Serial.print(" Z: ");
  Serial.print(wristCalibrationOffsetZ, 2);
  Serial.println(" m/s²");
  
  float gravityMagnitude = sqrt(wristCalibrationOffsetX*wristCalibrationOffsetX +
                               wristCalibrationOffsetY*wristCalibrationOffsetY +
                               wristCalibrationOffsetZ*wristCalibrationOffsetZ);
  Serial.print("Gravity magnitude: ");
  Serial.print(gravityMagnitude, 2);
  Serial.println(" m/s²");
  
  float gyroStability = abs(gyroAvgX) + abs(gyroAvgY) + abs(gyroAvgZ);
  if (gyroStability < 0.5) {
    Serial.println("Gyro stability: GOOD (device was still)");
  } else {
    Serial.println("Gyro stability: POOR (device moved during calibration)");
  }
}

// ========== MOTION PROCESSING ==========

// Classify current activity (simplified)
ActivityState classifyActivity(float accelMag, float gyroMag) {
  // Exponential smoothing
  avgAccel = 0.8 * avgAccel + 0.2 * accelMag;
  avgGyro  = 0.8 * avgGyro  + 0.2 * gyroMag;
  Serial.print("Accel: ");
  Serial.println(avgAccel);
  Serial.print("Gyro: ");
  Serial.println(avgGyro);

  if (avgAccel < 10 && avgGyro < 15)
    return ACTIVITY_IDLE;

  if (avgAccel < 20 && avgGyro < 25)
    return ACTIVITY_WALKING;

  if (avgAccel >= 20 || avgGyro >= 25)
    return ACTIVITY_RUNNING;

  return ACTIVITY_UNKNOWN;
}

// Detect step using wrist swing pattern
bool detectWristStep(float accelX, float accelY, float accelZ) {
  // AC motion magnitude (gravity already removed)
  float signal = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);

  float delta = signal - prevStepSignal;
  prevStepSignal = signal;

  // Wrist-appropriate thresholds
  if (delta > 0.25 && signal > 0.6) {
    if (millis() - lastStepDetectedTime > 350) { 
      lastStepDetectedTime = millis();
      return true;
    }
  }
  return false;
}

// Enhanced fall detection for wrist (simplified)
bool detectWristFall(float accelMag, float gyroMag) {
  static bool impactDetected = false;
  static unsigned long impactTime = 0;

  if (accelMag > 22.0 && gyroMag > 4.0) {
    impactDetected = true;
    impactTime = millis();
  }

  if (impactDetected && (millis() - impactTime < 800)) {
    if (accelMag < 4.0) {  // post-fall stillness
      impactDetected = false;
      return true;
    }
  }

  if (millis() - impactTime > 1500) {
    impactDetected = false;
  }

  return false;
}


// Check for post-fall stillness
void checkPostFallStillness(float accelMagnitude, float gyroMagnitude) {
  
  unsigned long fallDuration = millis() - fallDetectionStart;
  
  if (fallDuration > 2000 && fallDuration < 10000) {
    if (accelMagnitude < FALL_STILLNESS_THRESHOLD && gyroMagnitude < 0.5) {
      Serial.println("[MPU6050] Post-fall stillness detected");
      
      static unsigned long stillnessStart = 0;
      if (stillnessStart == 0) {
        stillnessStart = millis();
      } else if (millis() - stillnessStart > POST_FALL_STILL_TIME) {
        Serial.println("[MPU6050] CONFIRMED FALL - User motionless!");
        stillnessStart = 0;
        if (!sosActive) {
          activateSOS();
        }
      }
    } else {
      Serial.println("[MPU6050] Fall alert canceled - movement detected");
    }
  }
}

// Update sleep state based on prolonged stillness
void updateSleepState(float accelMag, float gyroMag) {
  if (!isBeingWorn()) {
    stillnessCounter = 0;
    isSleeping = false;
    return;
  }

  if (accelMag < 0.15 && gyroMag < 0.2) {
    stillnessCounter++;
  } else {
    stillnessCounter = 0;
    isSleeping = false;
  }

  static unsigned long stillnessStart = 0;

  if (accelMag < 0.15 && gyroMag < 0.2) {
    if (stillnessStart == 0) stillnessStart = millis();
    if (millis() - stillnessStart > 600000) { // 10 minutes
      isSleeping = true;
      Serial.println("[MPU6050] Sleep mode detected");
    }
  } else {
    stillnessStart = 0;
    isSleeping = false;
  }
}

// ========== MAIN READ FUNCTION ==========

// Read MPU6050 data with wrist-specific processing
void readMPU6050Data() {
  if (!mpu6050Detected) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastMotionRead >= MOTION_READ_INTERVAL) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    lastMotionRead = currentTime;
    
    float accelX = a.acceleration.x;
    float accelY = a.acceleration.y;
    float accelZ = a.acceleration.z;
    
    if (isCalibrated) {
      accelX -= wristCalibrationOffsetX;
      accelY -= wristCalibrationOffsetY;
      accelZ -= wristCalibrationOffsetZ;
    }
    
    float accelMagnitude = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    float gyroMagnitude = sqrt(g.gyro.x*g.gyro.x + g.gyro.y*g.gyro.y + g.gyro.z*g.gyro.z);
    lastAccelMagnitude = accelMagnitude;
    
    ActivityState newActivity = classifyActivity(accelMagnitude, gyroMagnitude);
    if (newActivity != currentActivity) {
      currentActivity = newActivity;
    }
    
    // Step detection only during walking/running
    if (currentActivity == ACTIVITY_WALKING ||
        currentActivity == ACTIVITY_RUNNING) {
      if (detectWristStep(accelX, accelY, accelZ)) {
        stepCount++;
        stepsToday = stepCount;
        lastStepTime = millis();
        
        static unsigned long lastStepLog = 0;
        if (millis() - lastStepLog > 5000) {
          Serial.print("[MPU6050] Step ");
          Serial.print(stepCount);
          Serial.print(" (Activity: ");
          Serial.print(activityToString(currentActivity));
          Serial.println(")");
          lastStepLog = millis();
        }
      }
    }
        
    if (!sosActive && detectWristFall(accelMagnitude, gyroMagnitude)) {
      activateSOS();
    }
    
    checkPostFallStillness(accelMagnitude, gyroMagnitude);
    
    updateSleepState(accelMagnitude, gyroMagnitude);
    
    static unsigned long lastDebugOutput = 0;
    if (millis() - lastDebugOutput > 30000) {
      Serial.print("[MPU6050] Status - Steps: ");
      Serial.print(stepsToday);
      Serial.print(", Activity: ");
      Serial.print(activityToString(currentActivity));
      Serial.print(", Sleep: ");
      Serial.print(isSleeping ? "YES" : "NO");
      Serial.print(", Accel: ");
      Serial.print(accelMagnitude, 2);
      Serial.print(" m/s², Gyro: ");
      Serial.print(gyroMagnitude, 2);
      Serial.println(" rad/s");
      lastDebugOutput = millis();
    }
  }
}

// ========== UTILITY FUNCTIONS ==========

// Check if device is being worn
bool isBeingWorn() {
  return (lastAccelMagnitude > 8.0 && lastAccelMagnitude < 11.0);
}
// Check sleep status
bool isSleepingNow() {
  return isSleeping;
}

// Get wear status as string
String getWearStatus() {
  if (!mpu6050Detected) return "SENSOR NOT DETECTED";
  return isBeingWorn() ? "BEING WORN" : "NOT WORN";
}

// ========== DEBUG & INFO FUNCTIONS ==========

// Print comprehensive MPU6050 info
void printMPU6050Info() {
  if (!mpu6050Detected) {
    Serial.println("MPU6050: Not detected");
    return;
  }
  
  Serial.println("\n=== MPU6050 INFO (Wrist Wearable) ===");
  
  sensors_event_t a, g, temp;
  if (mpu.getEvent(&a, &g, &temp)) {
    Serial.println("RAW SENSOR VALUES:");
    Serial.print("  Accel - X: ");
    Serial.print(a.acceleration.x, 2);
    Serial.print("  Y: ");
    Serial.print(a.acceleration.y, 2);
    Serial.print("  Z: ");
    Serial.print(a.acceleration.z, 2);
    Serial.println(" m/s²");
    
    Serial.print("  Gyro  - X: ");
    Serial.print(g.gyro.x, 2);
    Serial.print("  Y: ");
    Serial.print(g.gyro.y, 2);
    Serial.print("  Z: ");
    Serial.print(g.gyro.z, 2);
    Serial.println(" rad/s");
    
    Serial.print("  Temp: ");
    Serial.print(temp.temperature, 1);
    Serial.println("°C");
    
    if (isCalibrated) {
      Serial.println("\nCALIBRATED VALUES:");
      Serial.print("  Accel - X: ");
      Serial.print(a.acceleration.x - wristCalibrationOffsetX, 2);
      Serial.print("  Y: ");
      Serial.print(a.acceleration.y - wristCalibrationOffsetY, 2);
      Serial.print("  Z: ");
      Serial.print(a.acceleration.z - wristCalibrationOffsetZ, 2);
      Serial.println(" m/s²");
      
      float calAccelX = a.acceleration.x - wristCalibrationOffsetX;
      float calAccelY = a.acceleration.y - wristCalibrationOffsetY;
      float calAccelZ = a.acceleration.z - wristCalibrationOffsetZ;
      float calMagnitude = sqrt(calAccelX*calAccelX + calAccelY*calAccelY + calAccelZ*calAccelZ);
      
      Serial.print("  Magnitude: ");
      Serial.print(calMagnitude, 2);
      Serial.println(" m/s²");
    }
    
    Serial.println("\nSTATUS:");
    Serial.print("  Steps today: ");
    Serial.println(stepsToday);
    Serial.print("  Current activity: ");
    Serial.println(activityToString(currentActivity));
    Serial.print("  Wear status: ");
    Serial.println(getWearStatus());
    Serial.print("  Calibrated: ");
    Serial.println(isCalibrated ? "YES" : "NO");
    Serial.print("  Sleeping: ");
    Serial.println(isSleeping ? "YES" : "NO");
    Serial.print("  Stillness counter: ");
    Serial.println(stillnessCounter);
    
    if (isCalibrated) {
      Serial.print("  Calibration offsets - X: ");
      Serial.print(wristCalibrationOffsetX, 2);
      Serial.print("  Y: ");
      Serial.print(wristCalibrationOffsetY, 2);
      Serial.print("  Z: ");
      Serial.print(wristCalibrationOffsetZ, 2);
      Serial.println(" m/s²");
    }
  }
  
  Serial.println("=====================================\n");
}

// Reset step count
void resetStepCount() {
  stepCount = 0;
  stepsToday = 0;
  Serial.println("[MPU6050] Step count reset to 0");
}

// Get current activity as string
String getCurrentActivity() {
  return activityToString(currentActivity);
}

// Get step count
int getStepCount() {
  return stepCount;
}

// Get steps today
int getStepsToday() {
  return stepsToday;
}

// Get if wrist is calibrated
bool isWristCalibrated() {
  return isCalibrated;
}