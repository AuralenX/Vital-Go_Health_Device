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
unsigned long lastShakeTime = 0;

// Step detection variables
int stepBuffer[10];
int stepBufferIndex = 0;
float stepPatternConfidence = 0;
unsigned long lastValidStepTime = 0;

// Fall detection variables
bool potentialFallDetected = false;
unsigned long fallDetectionStart = 0;
float fallConfidence = 0;

// Activity classification
ActivityState currentActivity = ACTIVITY_IDLE;

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
  
  for (int i = 0; i < 10; i++) {
    stepBuffer[i] = 0;
  }
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

// Classify current activity
ActivityState classifyActivity(float accelMagnitude, float gyroMagnitude) {
  static unsigned long lastActivityUpdate = 0;
  static float avgAccel = 0, avgGyro = 0;
  static int sampleCount = 0;
  
  avgAccel = (avgAccel * sampleCount + accelMagnitude) / (sampleCount + 1);
  avgGyro = (avgGyro * sampleCount + gyroMagnitude) / (sampleCount + 1);
  sampleCount = min(sampleCount + 1, 20);
  
  if (millis() - lastActivityUpdate < 2000) {
    return currentActivity;
  }
  
  lastActivityUpdate = millis();
  
  if (avgAccel < 0.5 && avgGyro < 1.0) {
    return ACTIVITY_IDLE;
  } else if (avgAccel > 2.0 && avgGyro > 3.0) {
    return ACTIVITY_SHAKING;
  } else if (avgAccel > 1.0 && avgGyro > 2.0 && avgAccel < 3.0) {
    return ACTIVITY_WALKING;
  } else if (avgAccel > 3.0 && avgGyro > 4.0) {
    return ACTIVITY_RUNNING;
  } else {
    return ACTIVITY_UNKNOWN;
  }
}

// Detect step using pattern recognition
bool detectWristStep(float accelMagnitude, float gyroMagnitude, ActivityState activity) {
  if (activity != ACTIVITY_WALKING && activity != ACTIVITY_RUNNING) {
    return false;
  }
  
  stepBuffer[stepBufferIndex] = (accelMagnitude > ACCEL_THRESHOLD) ? 1 : 0;
  stepBufferIndex = (stepBufferIndex + 1) % 10;
  
  int patternScore = 0;
  for (int i = 0; i < 8; i++) {
    if (stepBuffer[i] != stepBuffer[i+1]) {
      patternScore++;
    }
  }
  
  if (patternScore >= 5 && accelMagnitude > ACCEL_THRESHOLD) {
    if (gyroMagnitude > 1.0) {
      stepPatternConfidence = min(stepPatternConfidence + 0.2, 1.0);
      
      if (stepPatternConfidence > 0.6 && millis() - lastValidStepTime > STEP_DEBOUNCE) {
        lastValidStepTime = millis();
        return true;
      }
    }
  } else {
    stepPatternConfidence = max(stepPatternConfidence - 0.1, 0.0);
  }
  
  return false;
}

// Enhanced fall detection for wrist
bool detectWristFall(float accelMagnitude, float gyroMagnitude, float gyroX, float gyroY, float gyroZ) {
  static unsigned long lastFallCheck = 0;
  
  if (millis() - lastFallCheck < 100) {
    return false;
  }
  lastFallCheck = millis();
  
  bool highImpact = accelMagnitude > FALL_IMPACT_THRESHOLD;
  bool highRotation = gyroMagnitude > 4.0;
  bool freeFall = accelMagnitude < FREE_FALL_THRESHOLD_HIGH && 
                  accelMagnitude > FREE_FALL_THRESHOLD_LOW;
  
  static float lastOrientationZ = 0;
  float orientationChange = abs(wristCalibrationOffsetZ - lastOrientationZ);
  lastOrientationZ = wristCalibrationOffsetZ;
  
  if (highImpact && highRotation) {
    fallConfidence += 0.4;
  } else if (freeFall && highRotation) {
    fallConfidence += 0.3;
  } else if (orientationChange > 5.0 && highImpact) {
    fallConfidence += 0.3;
  } else {
    fallConfidence = max(fallConfidence - 0.1, 0.0);
  }
  
  if (fallConfidence > 0.7 && !potentialFallDetected) {
    potentialFallDetected = true;
    fallDetectionStart = millis();
    Serial.println("[MPU6050] POTENTIAL FALL DETECTED!");
    Serial.print("  Impact: ");
    Serial.print(accelMagnitude, 1);
    Serial.print(" m/s², Rotation: ");
    Serial.print(gyroMagnitude, 1);
    Serial.println(" rad/s");
    return true;
  }
  
  if (potentialFallDetected && millis() - fallDetectionStart > 5000) {
    potentialFallDetected = false;
    fallConfidence = 0;
  }
  
  return false;
}

// Check for post-fall stillness
void checkPostFallStillness(float accelMagnitude, float gyroMagnitude) {
  if (!potentialFallDetected) return;
  
  unsigned long fallDuration = millis() - fallDetectionStart;
  
  if (fallDuration > 2000 && fallDuration < 10000) {
    if (accelMagnitude < FALL_STILLNESS_THRESHOLD && gyroMagnitude < 0.5) {
      Serial.println("[MPU6050] Post-fall stillness detected");
      
      static unsigned long stillnessStart = 0;
      if (stillnessStart == 0) {
        stillnessStart = millis();
      } else if (millis() - stillnessStart > POST_FALL_STILL_TIME) {
        Serial.println("[MPU6050] CONFIRMED FALL - User motionless!");
        potentialFallDetected = false;
        stillnessStart = 0;
        fallConfidence = 0;
      }
    } else {
      Serial.println("[MPU6050] Fall alert canceled - movement detected");
      potentialFallDetected = false;
      fallConfidence = 0;
    }
  }
  
  if (fallDuration > 10000) {
    potentialFallDetected = false;
    fallConfidence = 0;
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
    
    ActivityState newActivity = classifyActivity(accelMagnitude, gyroMagnitude);
    if (newActivity != currentActivity) {
      currentActivity = newActivity;
    }
    
    if (detectWristStep(accelMagnitude, gyroMagnitude, currentActivity)) {
      stepCount++;
      stepsToday = stepCount;
      lastStepTime = millis();
      
      static unsigned long lastStepLog = 0;
      if (millis() - lastStepLog > 5000) {
        Serial.print("[MPU6050] Step ");
        Serial.print(stepCount);
        Serial.print(" (Confidence: ");
        Serial.print(stepPatternConfidence, 2);
        Serial.print(", Activity: ");
        Serial.print(activityToString(currentActivity));
        Serial.println(")");
        lastStepLog = millis();
      }
    }
    
    bool fallDetected = detectWristFall(accelMagnitude, gyroMagnitude, 
                                        g.gyro.x, g.gyro.y, g.gyro.z);
    
    checkPostFallStillness(accelMagnitude, gyroMagnitude);
    
    static unsigned long lastDebugOutput = 0;
    if (millis() - lastDebugOutput > 30000) {
      Serial.print("[MPU6050] Status - Steps: ");
      Serial.print(stepsToday);
      Serial.print(", Activity: ");
      Serial.print(activityToString(currentActivity));
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
  if (!mpu6050Detected) return false;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float accelMagnitude = sqrt(a.acceleration.x*a.acceleration.x + 
                              a.acceleration.y*a.acceleration.y + 
                              a.acceleration.z*a.acceleration.z);
  
  return (accelMagnitude > 8.5 && accelMagnitude < 10.5);
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
    Serial.print("  Step confidence: ");
    Serial.println(stepPatternConfidence, 2);
    Serial.print("  Fall confidence: ");
    Serial.println(fallConfidence, 2);
    Serial.print("  Wear status: ");
    Serial.println(getWearStatus());
    Serial.print("  Calibrated: ");
    Serial.println(isCalibrated ? "YES" : "NO");
    
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