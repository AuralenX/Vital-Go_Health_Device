// sensor_monitor.ino
#include "bitmaps.h"

// Use extern for all variables
extern bool bmp280Detected;
extern bool mpu6050Detected;
extern bool max30102Detected;
extern bool sosActive;
extern int heartRate;
extern int bloodOxygen;
extern float bodyTemperature;
extern int bloodPressureSystolic;
extern int bloodPressureDiastolic;
extern int respiratoryRate;
extern int sleepQuality;
extern int stepsToday;
extern int batteryLevel;
extern int signalStrength;
extern Adafruit_MPU6050 mpu;
extern unsigned long lastSensorUpdate;
extern unsigned long lastMAX30102Read;

// Function declarations from other files
void readBMP280Data();
void readMPU6050Data();
void readMAX30102Data();
bool hasMinimumSensors();
void printSensorStatus();
void printBMP280Info();
void printMPU6050Info();
void printMAX30102Info();
void printAllSensorData();
void printHelp();
void calibrateMPU6050();

// Update all sensor data
void updateSensorData() {
  unsigned long currentTime = millis();

  if (currentTime - lastMAX30102Read >= MAX30102_INTERVAL) {
    readMAX30102Data();
    lastMAX30102Read = currentTime;
  }
  
  if (currentTime - lastSensorUpdate >= DATA_UPDATE_INTERVAL) {
    readBMP280Data();
    readMPU6050Data();
    calculateDerivedVitals();
    checkVitalAlerts();
    lastSensorUpdate = currentTime;
  }
}

// Calculate derived vitals
void calculateDerivedVitals() {
  
  // Estimate sleep quality
  static int movementCount = 0;
  static unsigned long lastSleepCheck = 0;
  
  if (millis() - lastSleepCheck > 60000) {
    if (movementCount < 10) {
      sleepQuality = 90 - (movementCount * 2);
    } else {
      sleepQuality = 50;
    }
    
    movementCount = 0;
    lastSleepCheck = millis();
    
    Serial.print("[SYSTEM] Sleep quality: ");
    Serial.println(sleepQuality);
  }
  
  // Count movements during sleep hours
  int hour = (millis() / 3600000) % 24;
  if ((hour >= 22 || hour <= 6) && mpu6050Detected) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float movement = abs(a.acceleration.x) + abs(a.acceleration.y) + abs(a.acceleration.z);
    if (movement > 0.5) {
      movementCount++;
    }
  }
  
  // Simulate battery drain
  static unsigned long startTime = millis();
  unsigned long elapsedHours = (millis() - startTime) / 3600000;
  batteryLevel = 75 - (elapsedHours * 2);
  batteryLevel = constrain(batteryLevel, 0, 100);
  
  // Simulate signal strength
  signalStrength = 2 + random(-1, 2);
  signalStrength = constrain(signalStrength, 0, 5);
}

// Check for vital alerts
void checkVitalAlerts() {
  if (!hasMinimumSensors()) return;
  
  bool criticalAlert = false;
  String alertMessage = "";
  
  // Heart rate alerts
  if (max30102Detected && heartRate > 0) {
    if (heartRate > HR_CRITICAL_HIGH || heartRate < HR_CRITICAL_LOW) {
      criticalAlert = true;
      alertMessage = "CRITICAL HR: " + String(heartRate) + " BPM";
    } else if (heartRate > HR_WARNING_HIGH || heartRate < HR_WARNING_LOW) {
      Serial.print("[WARNING] Abnormal HR: ");
      Serial.println(heartRate);
    }
  }
  
  // SpO2 alerts
  if (max30102Detected && bloodOxygen > 0) {
    if (bloodOxygen < SPO2_CRITICAL) {
      criticalAlert = true;
      alertMessage = "CRITICAL SpO2: " + String(bloodOxygen) + "%";
    } else if (bloodOxygen < SPO2_WARNING) {
      Serial.print("[WARNING] Low SpO2: ");
      Serial.println(bloodOxygen);
    }
  }
  
  // Temperature alerts
  if (bmp280Detected && bodyTemperature > 0) {
    if (bodyTemperature > TEMP_FEVER) {
      criticalAlert = true;
      alertMessage = "FEVER: " + String(bodyTemperature, 1) + "°C";
    }
  }
  
  // Respiratory rate alerts (now from PPG)
  if (max30102Detected && respiratoryRate > 0) {
    if (respiratoryRate > RESP_RATE_MAX || respiratoryRate < RESP_RATE_MIN) {
      criticalAlert = true;
      alertMessage = "CRITICAL Resp Rate: " + String(respiratoryRate) + " BPM";
      Serial.print("[ALERT] Respiratory rate alert from PPG: ");
      Serial.println(respiratoryRate);
    }
  }
  
  // Blood pressure alerts
  if (max30102Detected && bloodPressureSystolic > 0) {
    if (bloodPressureSystolic > 180 || bloodPressureSystolic < 90 ||
        bloodPressureDiastolic > 120 || bloodPressureDiastolic < 60) {
      criticalAlert = true;
      alertMessage = "CRITICAL BP: " + String(bloodPressureSystolic) + 
                     "/" + String(bloodPressureDiastolic);
    }
  }
  
  if (criticalAlert) {
    Serial.print("[ALERT] ");
    Serial.println(alertMessage);
    // Uncomment if you have SOS functionality:
    // if (!sosActive) activateSOS();
  }
}

// Command interpreter for serial monitor
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "status") {
      printSensorStatus();
    }
    else if (command == "bmp280") {
      printBMP280Info();
    }
    else if (command == "mpu6050") {
      printMPU6050Info();
    }
    else if (command == "max30102") {
      printMAX30102Info();
    }
    else if (command == "data") {
      printAllSensorData();
    }
    else if (command == "calibrate") {
      calibrateMPU6050();
    }
    else if (command == "help") {
      printHelp();
    }
    else {
      Serial.println("Unknown command. Type 'help' for commands.");
    }
  }
}

// Print all sensor data
void printAllSensorData() {
  Serial.println("\n=== ALL SENSOR DATA ===");
  Serial.print("Body temp: ");
  Serial.print(bodyTemperature);
  Serial.println("°C");
  
  Serial.print("Heart rate: ");
  Serial.print(heartRate);
  Serial.println(" BPM");
  
  Serial.print("SpO2: ");
  Serial.print(bloodOxygen);
  Serial.println("%");
  
  Serial.print("BP: ");
  Serial.print(bloodPressureSystolic);
  Serial.print("/");
  Serial.print(bloodPressureDiastolic);
  Serial.println(" mmHg");
  
  Serial.print("Resp rate: ");
  Serial.print(respiratoryRate);
  Serial.println(" BPM");
  
  Serial.print("Steps: ");
  Serial.println(stepsToday);
  
  Serial.print("Sleep quality: ");
  Serial.println(sleepQuality);
  
  Serial.print("Battery: ");
  Serial.print(batteryLevel);
  Serial.println("%");
  
  Serial.println("=======================\n");
}

// Print help
void printHelp() {
  Serial.println("\n=== SERIAL COMMANDS ===");
  Serial.println("status    - Show sensor detection status");
  Serial.println("bmp280    - Show BMP280 (temp/pressure) data");
  Serial.println("mpu6050   - Show MPU6050 (motion) data");
  Serial.println("max30102  - Show MAX30102 (HR/SpO2) data");
  Serial.println("data      - Show all sensor readings");
  Serial.println("calibrate - Calibrate MPU6050");
  Serial.println("help      - Show this help");
  Serial.println("========================\n");
}