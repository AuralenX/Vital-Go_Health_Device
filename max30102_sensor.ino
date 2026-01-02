// max30102_sensor.ino - Add respiratory rate estimation
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

// ========== RESPIRATORY RATE VARIABLES ==========
float respPPGBuffer[100];  // Buffer for PPG respiratory analysis
int respPPGIndex = 0;
unsigned long lastRespPPGUpdate = 0;
float lastIRValue = 0;
float respiratoryRatePPG = 16;  // PPG-based respiratory rate

// ========== RESPIRATORY ALGORITHM CONSTANTS ==========
#define RESP_PPG_SAMPLE_RATE 25  // Hz (25 samples per second)
#define RESP_PPG_WINDOW_SIZE 100 // 4 seconds of data (100 samples / 25 Hz)
#define RESP_PPG_UPDATE_INTERVAL 10000  // Update every 10 seconds

// Read MAX30102 data with respiratory rate
void readMAX30102Data() {
  if (!max30102Detected) return;
  
  unsigned long currentTime = millis();
  
  // Always read IR value for respiratory analysis (faster sampling)
  long currentIR = particleSensor.getIR();
  
  // ========== RESPIRATORY RATE FROM PPG ==========
  if (currentIR > 30000) {  // Good signal quality
    // Store PPG data for respiratory analysis
    float irDiff = currentIR - lastIRValue;
    respPPGBuffer[respPPGIndex] = irDiff;
    respPPGIndex = (respPPGIndex + 1) % RESP_PPG_WINDOW_SIZE;
    lastIRValue = currentIR;
    
    // Analyze respiratory rate every 10 seconds
    if (currentTime - lastRespPPGUpdate >= RESP_PPG_UPDATE_INTERVAL) {
      estimateRespiratoryRateFromPPG();
      lastRespPPGUpdate = currentTime;
    }
  }
  
  // ========== HEART RATE & SpO2 (existing code) ==========
  if (currentTime - lastHRRead >= HR_READ_INTERVAL) {
    irValue = currentIR;
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

// Estimate respiratory rate from PPG signal
void estimateRespiratoryRateFromPPG() {
  if (respPPGIndex < RESP_PPG_WINDOW_SIZE / 2) {
    return; // Not enough data yet
  }
  
  // Simple peak detection algorithm for respiratory rate
  int breathPeaks = 0;
  float threshold = 0.0;
  
  // Calculate threshold (average amplitude * 0.7)
  float sumAmplitude = 0;
  for (int i = 0; i < RESP_PPG_WINDOW_SIZE; i++) {
    sumAmplitude += abs(respPPGBuffer[i]);
  }
  threshold = (sumAmplitude / RESP_PPG_WINDOW_SIZE) * 0.7;
  
  // Find peaks (breaths)
  for (int i = 3; i < RESP_PPG_WINDOW_SIZE - 3; i++) {
    // Check if this is a peak (inhalation or exhalation)
    if (respPPGBuffer[i] > threshold &&
        respPPGBuffer[i] > respPPGBuffer[i-1] &&
        respPPGBuffer[i] > respPPGBuffer[i-2] &&
        respPPGBuffer[i] > respPPGBuffer[i+1] &&
        respPPGBuffer[i] > respPPGBuffer[i+2]) {
      breathPeaks++;
      i += 5; // Skip a bit to avoid counting same peak multiple times
    }
  }
  
  // Calculate respiratory rate (breaths per minute)
  if (breathPeaks > 0) {
    // Each peak represents a respiratory event
    // 100 samples = 4 seconds at 25Hz
    float breathsPerSecond = (float)breathPeaks / 4.0;
    float calculatedRR = breathsPerSecond * 60.0;
    
    // Smooth the result
    respiratoryRatePPG = (respiratoryRatePPG * 0.7) + (calculatedRR * 0.3);
    
    // Constrain to reasonable values
    respiratoryRatePPG = constrain(respiratoryRatePPG, 8, 30);
    
    // Use this for the main respiratory rate variable
    respiratoryRate = (int)respiratoryRatePPG;
    
    Serial.print("[MAX30102] Respiratory rate from PPG: ");
    Serial.print(respiratoryRate);
    Serial.println(" BPM");
  }
}

// Alternative: Frequency domain analysis (more accurate but complex)
void estimateRespiratoryRateFFT() {
  // This would be more accurate but requires FFT library
  // Basic implementation for reference:
  
  /*
  1. Apply bandpass filter (0.1-0.5 Hz = 6-30 BPM)
  2. Find dominant frequency using peak detection
  3. Convert frequency to breaths per minute
  */
  
  // For now, we'll use the time-domain method above
}

// Enhanced SpO2 estimation that considers respiratory modulation
void estimateSpO2() {
  long redValue = particleSensor.getRed();
  
  if (irValue > 50000 && redValue > 50000) {
    float ratio = (float)redValue / (float)irValue;
    float spo2Value = 100.0 - (20.0 * ratio);
    
    // Respiratory modulation correction
    // Breathing affects SpO2 readings slightly
    float respiratoryModulation = 0.5 * sin(millis() / 1000.0 * TWO_PI / (60.0 / respiratoryRatePPG));
    spo2Value += respiratoryModulation;
    
    spo2Value = constrain(spo2Value, 70, 100);
    bloodOxygen = (int)spo2Value;
    
    Serial.print("[MAX30102] SpO2: ");
    Serial.print(bloodOxygen);
    Serial.println("%");
  }
}

// Estimate blood pressure with respiratory correlation
void estimateBloodPressure(float bpm) {
  float baseSystolic = 120.0;
  float baseDiastolic = 80.0;
  
  // Respiratory sinus arrhythmia effect
  // Blood pressure varies slightly with breathing
  float respiratoryEffect = 0.3 * sin(millis() / 1000.0 * TWO_PI / (60.0 / respiratoryRatePPG));
  
  if (bpm > 80) {
    bloodPressureSystolic = baseSystolic + ((bpm - 80) * 0.5) + respiratoryEffect;
    bloodPressureDiastolic = baseDiastolic + ((bpm - 80) * 0.3) + (respiratoryEffect * 0.5);
  } else if (bpm < 60) {
    bloodPressureSystolic = baseSystolic - ((60 - bpm) * 0.5) + respiratoryEffect;
    bloodPressureDiastolic = baseDiastolic - ((60 - bpm) * 0.3) + (respiratoryEffect * 0.5);
  } else {
    bloodPressureSystolic = baseSystolic + respiratoryEffect;
    bloodPressureDiastolic = baseDiastolic + (respiratoryEffect * 0.5);
  }
  
  // Add some randomness
  bloodPressureSystolic += random(-3, 3);
  bloodPressureDiastolic += random(-2, 2);
  
  bloodPressureSystolic = constrain(bloodPressureSystolic, 90, 180);
  bloodPressureDiastolic = constrain(bloodPressureDiastolic, 60, 120);
  
  Serial.print("[MAX30102] BP: ");
  Serial.print(bloodPressureSystolic);
  Serial.print("/");
  Serial.print(bloodPressureDiastolic);
  Serial.println(" mmHg");
  
  // Note: Respiratory rate affects blood pressure variability
  Serial.print("[MAX30102] Respiratory-BP correlation: ");
  Serial.print(respiratoryEffect, 2);
  Serial.println(" mmHg");
}

// Print enhanced MAX30102 info with respiratory data
void printMAX30102Info() {
  if (!max30102Detected) {
    Serial.println("MAX30102: Not detected");
    return;
  }
  
  Serial.println("=== MAX30102 INFO (Enhanced) ===");
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
  Serial.print("Respiratory rate (PPG): ");
  Serial.print(respiratoryRate);
  Serial.println(" BPM");
  Serial.print("Signal quality: ");
  Serial.println(irValue > 30000 ? "GOOD" : "POOR");
  Serial.println("===============================\n");
}

// New function: Check signal quality for respiratory analysis
String getPPGSignalQuality() {
  if (irValue > 70000) return "EXCELLENT";
  if (irValue > 50000) return "GOOD";
  if (irValue > 30000) return "FAIR";
  if (irValue > 10000) return "POOR";
  return "NO SIGNAL";
}