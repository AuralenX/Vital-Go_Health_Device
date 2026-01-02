#include "bitmaps.h"
#include <MAX30105.h>
#include "heartRate.h"
#include "spo2_algorithm.h"

// Use extern
extern MAX30105 particleSensor;
extern bool max30102Detected;
extern long irValue;
extern byte rates[4];
extern byte rateSpot;
extern long lastBeat;
extern float beatsPerMinute;
extern int beatAvg;
extern int heartRate;
extern int bloodOxygen;
extern int bloodPressureSystolic;
extern int bloodPressureDiastolic;
extern unsigned long lastHRRead;

// Read MAX30102 data
void readMAX30102Data() {
  if (!max30102Detected) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastHRRead >= HR_READ_INTERVAL) {
    irValue = particleSensor.getIR();
    lastHRRead = currentTime;
    
    if (irValue > 50000) {
      // Heart rate detection
      if (checkForBeat(irValue) == true) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        
        beatsPerMinute = 60 / (delta / 1000.0);
        
        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= 4;
          
          beatAvg = 0;
          for (byte x = 0; x < 4; x++) {
            beatAvg += rates[x];
          }
          beatAvg /= 4;
          
          heartRate = (int)beatAvg * HR_CALIBRATION_FACTOR;
          
          Serial.print("[MAX30102] Heart rate: ");
          Serial.print(heartRate);
          Serial.println(" BPM");
          
          // Estimate blood pressure
          estimateBloodPressure(beatsPerMinute);
        }
      }
      
      // SpO2 estimation
      estimateSpO2();
      
    } else {
      Serial.println("[MAX30102] Finger not detected");
      heartRate = 0;
      bloodOxygen = 0;
    }
  }
}

// Estimate SpO2
void estimateSpO2() {
  long redValue = particleSensor.getRed();
  
  if (irValue > 50000 && redValue > 50000) {
    float ratio = (float)redValue / (float)irValue;
    float spo2Value = 100.0 - (20.0 * ratio);
    
    spo2Value = constrain(spo2Value, 70, 100);
    bloodOxygen = (int)spo2Value;
    
    Serial.print("[MAX30102] SpO2: ");
    Serial.print(bloodOxygen);
    Serial.println("%");
  }
}

// Estimate blood pressure
void estimateBloodPressure(float bpm) {
  float baseSystolic = 120.0;
  float baseDiastolic = 80.0;
  
  if (bpm > 80) {
    bloodPressureSystolic = baseSystolic + ((bpm - 80) * 0.5);
    bloodPressureDiastolic = baseDiastolic + ((bpm - 80) * 0.3);
  } else if (bpm < 60) {
    bloodPressureSystolic = baseSystolic - ((60 - bpm) * 0.5);
    bloodPressureDiastolic = baseDiastolic - ((60 - bpm) * 0.3);
  }
  
  // Add some randomness
  bloodPressureSystolic += random(-5, 5);
  bloodPressureDiastolic += random(-3, 3);
  
  bloodPressureSystolic = constrain(bloodPressureSystolic, 90, 180);
  bloodPressureDiastolic = constrain(bloodPressureDiastolic, 60, 120);
  
  Serial.print("[MAX30102] BP: ");
  Serial.print(bloodPressureSystolic);
  Serial.print("/");
  Serial.print(bloodPressureDiastolic);
  Serial.println(" mmHg");
}

// Print MAX30102 info
void printMAX30102Info() {
  if (!max30102Detected) {
    Serial.println("MAX30102: Not detected");
    return;
  }
  
  Serial.println("=== MAX30102 INFO ===");
  Serial.print("IR value: ");
  Serial.println(particleSensor.getIR());
  Serial.print("Red value: ");
  Serial.println(particleSensor.getRed());
  Serial.print("Heart rate: ");
  Serial.print(heartRate);
  Serial.println(" BPM");
  Serial.print("SpO2: ");
  Serial.print(bloodOxygen);
  Serial.println("%");
  Serial.print("Blood pressure: ");
  Serial.print(bloodPressureSystolic);
  Serial.print("/");
  Serial.print(bloodPressureDiastolic);
  Serial.println(" mmHg");
  Serial.println("=====================\n");
}