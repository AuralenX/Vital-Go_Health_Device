// All sensor integration
#include "bitmaps.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include "heartRate.h" 
#include "spo2_algorithm.h" 

// Sensor objects
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

// Sensor status flags
bool bmp280Detected = false;
bool mpu6050Detected = false;
bool max30102Detected = false;

// Sensor demo data variables
int heartRate = 78;
int bloodOxygen = 97;
float bodyTemperature = 36.5;
float ambientTemperature = 25.0;
int bloodPressureSystolic = 120;
int bloodPressureDiastolic = 80;
int respiratoryRate = 16;
int sleepQuality = 85;
int stepsToday = 4256;
int batteryLevel = 85;
int signalStrength = 4;

// Motion tracking variables
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
float accelThreshold = 1.5; 
int stepCount = 0;
unsigned long lastStepTime = 0;
const unsigned long STEP_DEBOUNCE = 300; 

// Respiratory tracking
float respBuffer[20]; 
int respIndex = 0;
unsigned long lastRespUpdate = 0;

// Heart rate tracking (for MAX30102)
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
long irValue;

// Timing variables
unsigned long lastHRRead = 0;
unsigned long lastTempRead = 0;
unsigned long lastMotionRead = 0;

// Initialize all sensors
void initSensors() {
  Serial.println("Initializing sensors...");
  Wire.begin();
  Wire.setClock(400000);
  
  // 1. Initialize BMP280 (Temperature/Pressure)
  if (bmp.begin(BMP280_ADDRESS)) {
    bmp280Detected = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280 initialized successfully");
  } else {
    Serial.println("BMP280 not found!");
  }
  
  // 2. Initialize MPU6050 (Motion)
  if (mpu.begin(MPU6050_ADDRESS)) {
    mpu6050Detected = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 initialized successfully");
    
    // Calibrate MPU6050
    calibrateMPU6050();
  } else {
    Serial.println("MPU6050 not found!");
  }
  
  // 3. Initialize MAX30102 (Heart Rate & SpO2)
  if (particleSensor.begin(Wire, I2C_SPEED_FAST, MAX30102_ADDRESS)) {
    max30102Detected = true;
    
    // Configure MAX30102
    byte ledBrightness = 0x3F;  
    byte sampleAverage = 4;     
    byte ledMode = 3;           
    int sampleRate = 400; 
    int pulseWidth = 411; 
    int adcRange = 4096;  
    
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    particleSensor.setPulseAmplitudeRed(0x2F);
    particleSensor.setPulseAmplitudeGreen(0);
    
    Serial.println("MAX30102 initialized successfully");
  } else {
    Serial.println("MAX30102 not found!");
  }
  
  // Initialize respiratory buffer
  for (int i = 0; i < 20; i++) {
    respBuffer[i] = 0;
  }
  printSensorStatus();
}

// Calibrate MPU6050
void calibrateMPU6050() {
  if (!mpu6050Detected) return;
  
  Serial.println("Calibrating MPU6050... Keep sensor still!");
  delay(1000);
  
  sensors_event_t a, g, temp;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    accelXSum += a.acceleration.x;
    accelYSum += a.acceleration.y;
    accelZSum += a.acceleration.z;
    delay(10);
  }
  
  lastAccelX = accelXSum / samples;
  lastAccelY = accelYSum / samples;
  lastAccelZ = accelZSum / samples;
  
  Serial.println("MPU6050 calibration complete");
  Serial.print("Offsets - X: ");
  Serial.print(lastAccelX);
  Serial.print(" Y: ");
  Serial.print(lastAccelY);
  Serial.print(" Z: ");
  Serial.println(lastAccelZ);
}

// Print sensor detection status
void printSensorStatus() {
  Serial.println("\n=== SENSOR STATUS ===");
  Serial.print("BMP280: ");
  Serial.println(bmp280Detected ? "DETECTED" : "NOT DETECTED");
  Serial.print("MPU6050: ");
  Serial.println(mpu6050Detected ? "DETECTED" : "NOT DETECTED");
  Serial.print("MAX30102: ");
  Serial.println(max30102Detected ? "DETECTED" : "NOT DETECTED");
  Serial.println("====================\n");
}

// Update all sensor readings
void updateSensorData() {
  unsigned long currentTime = millis();
  
  // Read ambient temperature from BMP280 (every 5 seconds)
  if (bmp280Detected && currentTime - lastTempRead >= TEMP_READ_INTERVAL) {
    ambientTemperature = bmp.readTemperature();
    lastTempRead = currentTime;
  }
  
  // Read motion data from MPU6050 (every 100ms)
  if (mpu6050Detected && currentTime - lastMotionRead >= MOTION_READ_INTERVAL) {
    readMotionData();
    lastMotionRead = currentTime;
  }
  
  // Read heart rate and SpO2 from MAX30102 (every 1 second)
  if (max30102Detected && currentTime - lastHRRead >= HR_READ_INTERVAL) {
    readHeartRateData();
    lastHRRead = currentTime;
  }
  
  // Calculate derived vitals
  calculateDerivedVitals();
}

// Read motion data from MPU6050
void readMotionData() {
  if (!mpu6050Detected) return;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate acceleration magnitude
  float accelX = a.acceleration.x - lastAccelX;
  float accelY = a.acceleration.y - lastAccelY;
  float accelZ = a.acceleration.z - lastAccelZ;
  float accelMagnitude = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
  
  // Step detection
  if (accelMagnitude > accelThreshold && 
      millis() - lastStepTime > STEP_DEBOUNCE) {
    stepCount++;
    stepsToday = stepCount;
    lastStepTime = millis();
    Serial.print("Step detected! Total: ");
    Serial.println(stepCount);
  }
  
  // Respiratory rate estimation (from chest movement)
  estimateRespiratoryRate(accelZ);
  
  // Fall detection
  checkFallDetection(accelMagnitude, g.gyro.x, g.gyro.y, g.gyro.z);
}

// Estimate respiratory rate from motion
void estimateRespiratoryRate(float chestMovement) {
  respBuffer[respIndex] = chestMovement;
  respIndex = (respIndex + 1) % 20;
  
  // Calculate respiratory rate every 5 seconds
  if (millis() - lastRespUpdate > 5000) {
    // Simple peak detection algorithm
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
      
      if (respiratoryRate < 8) respiratoryRate = 8;
      if (respiratoryRate > 30) respiratoryRate = 30;
    }
    
    lastRespUpdate = millis();
  }
}

// Check for falls
void checkFallDetection(float accelMagnitude, float gyroX, float gyroY, float gyroZ) {
  if (!mpu6050Detected) return; 
  
  static bool fallDetected = false;
  static unsigned long fallTime = 0;
  
  // Detect sudden impact (high acceleration)
  if (accelMagnitude > 4.0 && !fallDetected) {  
    fallDetected = true;
    fallTime = millis();
    Serial.println("Fall detected! Impact detected.");
  }
  
  // Check for post-fall stillness
  if (fallDetected && millis() - fallTime > 3000) {
    if (accelMagnitude < 0.5) {  
      if (!sosActive) {
        Serial.println("EMERGENCY: Fall confirmed! No movement detected.");
        activateSOS();
      }
    }
    fallDetected = false;
  }
}

// Read heart rate and SpO2 from MAX30102
void readHeartRateData() {
  if (!max30102Detected) return;
  
  irValue = particleSensor.getIR();
  
  if (irValue > 50000) { 
    // Check for heartbeat
    if (checkForBeat(irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      
      beatsPerMinute = 60 / (delta / 1000.0);
      
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        
        // Calculate average
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          beatAvg += rates[x];
        }
        beatAvg /= RATE_SIZE;
        
        heartRate = (int)beatAvg * HR_CALIBRATION_FACTOR;
        
        // Estimate blood pressure from pulse transit time
        estimateBloodPressure(beatsPerMinute);
        
        // Estimate SpO2
        estimateSpO2();
      }
    }
  } else {
    heartRate = 0;
    bloodOxygen = 0;
  }
}

// Estimate SpO2 from MAX30102
void estimateSpO2() {
  long redValue = particleSensor.getRed();
  
  if (irValue > 50000 && redValue > 50000) {
    // Calculate ratio for SpO2 estimation
    float ratio = (float)redValue / (float)irValue;
    float spo2Value = 100.0 - (20.0 * ratio);
    
    if (spo2Value > 100) spo2Value = 100;
    if (spo2Value < 70) spo2Value = 70;
    
    bloodOxygen = (int)spo2Value;
  }
}

// Estimate blood pressure (PPG-based estimation)
void estimateBloodPressure(float bpm) {
  // Simplified estimation based on heart rate
  float baseSystolic = 120.0;
  float baseDiastolic = 80.0;
  
  // Adjust based on heart rate (very rough estimation)
  if (bpm > 80) {
    bloodPressureSystolic = baseSystolic + ((bpm - 80) * 0.5);
    bloodPressureDiastolic = baseDiastolic + ((bpm - 80) * 0.3);
  } else if (bpm < 60) {
    bloodPressureSystolic = baseSystolic - ((60 - bpm) * 0.5);
    bloodPressureDiastolic = baseDiastolic - ((60 - bpm) * 0.3);
  } else {
    bloodPressureSystolic = baseSystolic;
    bloodPressureDiastolic = baseDiastolic;
  }
  
  // bloodPressureSystolic += random(-5, 5);
  // bloodPressureDiastolic += random(-3, 3);
  
  if (bloodPressureSystolic < 90) bloodPressureSystolic = 90;
  if (bloodPressureSystolic > 180) bloodPressureSystolic = 180;
  if (bloodPressureDiastolic < 60) bloodPressureDiastolic = 60;
  if (bloodPressureDiastolic > 120) bloodPressureDiastolic = 120;
}

// Calculate derived vitals (sleep quality, etc.)
void calculateDerivedVitals() {
  // Estimate body temperature (BMP280 reads ambient, so we estimate)
  bodyTemperature = ambientTemperature + TEMPERATURE_OFFSET;

  // Estimate sleep quality based on movement during sleep hours
  int hour = (millis() / 3600000) % 24;  // Simulated hour
  
  if (hour >= 22 || hour <= 6) {  
    static int movementCount = 0;
    static unsigned long lastSleepCheck = 0;
    
    if (millis() - lastSleepCheck > 60000) { 
      if (movementCount < 10) {
        sleepQuality = 90 - (movementCount * 2);
      } else {
        sleepQuality = 70;
      }
      
      movementCount = 0;
      lastSleepCheck = millis();
    }
    
    // Count movements during sleep
    sensors_event_t a, g, temp;
    if (mpu6050Detected) {
      mpu.getEvent(&a, &g, &temp);
      float movement = abs(a.acceleration.x) + abs(a.acceleration.y) + abs(a.acceleration.z);
      if (movement > 0.5) {
        movementCount++;
      }
    }
  }
  
  // Simulate battery drain (in real device, I will read from ADC)
  static unsigned long startTime = millis();
  unsigned long elapsedHours = (millis() - startTime) / 3600000;
  batteryLevel = 75 - (elapsedHours * 2); 
  if (batteryLevel < 0) batteryLevel = 0;
  
  // Simulate signal strength variation
  signalStrength = 2 + random(-1, 2);
  if (signalStrength < 0) signalStrength = 0;
  if (signalStrength > 5) signalStrength = 5;
}

// Check vital signs for emergencies
void checkVitalAlerts() {
  if (!bmp280Detected && !mpu6050Detected && !max30102Detected) {
    return;
  }
  
  bool criticalAlert = false;
  String alertMessage = "";
  
  // 1. HEART RATE ALERTS (requires MAX30102)
  if (max30102Detected && heartRate > 0) {
    if (heartRate > HR_CRITICAL_HIGH || heartRate < HR_CRITICAL_LOW) {
      criticalAlert = true;
      alertMessage = "CRITICAL HR: " + String(heartRate) + " BPM";
      Serial.println(alertMessage);
    } 
    else if (heartRate > HR_WARNING_HIGH || heartRate < HR_WARNING_LOW) {
      Serial.print("WARNING: Abnormal HR: ");
      Serial.println(heartRate);
    }
  }
  
  // 2. SPO2 ALERTS (requires MAX30102)
  if (max30102Detected && bloodOxygen > 0) {
    if (bloodOxygen < SPO2_CRITICAL) {
      criticalAlert = true;
      alertMessage = "CRITICAL SpO2: " + String(bloodOxygen) + "%";
      Serial.println(alertMessage);
    }
    else if (bloodOxygen < SPO2_WARNING) {
      Serial.print("WARNING: Low SpO2: ");
      Serial.println(bloodOxygen);
    }
  }
  
  // 3. TEMPERATURE ALERTS (requires BMP280)
  if (bmp280Detected && bodyTemperature > 0) {
    if (bodyTemperature > TEMP_FEVER) {
      criticalAlert = true;
      alertMessage = "FEVER: " + String(bodyTemperature, 1) + "°C";
      Serial.println(alertMessage);
    }
  }

  
  // 4. RESPIRATORY RATE ALERTS (requires MPU6050)
  if (mpu6050Detected && respiratoryRate > 0) {
    if (respiratoryRate > 30 || respiratoryRate < 8) {
      criticalAlert = true;
      alertMessage = "CRITICAL Resp Rate: " + String(respiratoryRate) + " BPM";
      Serial.println(alertMessage);
    }
  }
  
  // 5. BLOOD PRESSURE ALERTS (requires MAX30102)
  if (max30102Detected && bloodPressureSystolic > 0) {
    if (bloodPressureSystolic > 180 || bloodPressureSystolic < 90 ||
        bloodPressureDiastolic > 120 || bloodPressureDiastolic < 60) {
      criticalAlert = true;
      alertMessage = "CRITICAL BP: " + String(bloodPressureSystolic) + 
                    "/" + String(bloodPressureDiastolic) + " mmHg";
      Serial.println(alertMessage);
    }
  }

  if (criticalAlert && !sosActive) {
    activateSOS();
    Serial.print("Triggered by: ");
    Serial.println(alertMessage);
    Serial.print("Active sensors: ");
    if (max30102Detected) Serial.print("HR ");
    if (bmp280Detected) Serial.print("TEMP ");
    if (mpu6050Detected) Serial.print("MOTION ");
    Serial.println();
  }
}

bool hasMinimumSensors() {
  return (max30102Detected || bmp280Detected || mpu6050Detected);
}

// Get sensor status for display
String getSensorStatus() {
  if (!hasMinimumSensors()) {
    return "NO SENSORS";
  }
  
  String status = "";
  if (max30102Detected) status += "HR ";
  if (bmp280Detected) status += "TEMP ";
  if (mpu6050Detected) status += "MOTION ";
  
  return status;
}
// Update sensors data
void updateSimulatedData() {
  updateSensorData();
  checkVitalAlerts();
}