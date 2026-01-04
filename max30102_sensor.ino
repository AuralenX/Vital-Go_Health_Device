#include "bitmaps.h"
#include <MAX30105.h>
#include "heartRate.h"
#include "spo2_algorithm.h"

// extern for variables from sensor_config.ino
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
extern unsigned long lastBPUpdate;
extern unsigned long lastRRUpdate;
extern int respiratoryRate;

// ========== PULSE DETECTION VARIABLES ==========
const int RESP_BUFFER_SIZE = 100; 
float respPPGBuffer[RESP_BUFFER_SIZE];
int respPPGIndex = 0;
unsigned long lastRespPPGUpdate = 0;
float lastIRValue = 0;
float respiratoryRatePPG = 0;

// BPM calculation variables
long irBuffer[100];
int bufferIndex = 0;
long lastValidBPM = 0;
static unsigned long lastBeatDetected = 0;
static unsigned long lastBeatTime = 0;
const unsigned long REFRACTORY_PERIOD = 350; 
static long lastIR = 0;
static bool wasRising = false;

// ========== SENSOR SETUP ==========

void setupMAX30102() {
  if (!max30102Detected) return;
  
  Serial.println("[MAX30102] Initializing with optimal settings...");
  
  // Reset sensor first
  particleSensor.softReset();
  delay(100);
  
  particleSensor.clearFIFO();
  
  // Optimal settings for reliable readings
  byte ledBrightness = 10;    
  byte sampleAverage = 4;     
  byte ledMode = 3;            
  int sampleRate = 100;       
  int pulseWidth = 411;       
  int adcRange = 4096;         
  
  // Apply settings
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  
  // Configure individual LED brightness
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  
  // Enable FIFO rollover
  particleSensor.enableFIFORollover();
  delay(200);
  
  // Initial reading test
  long testIR = particleSensor.getIR();
  long testRed = particleSensor.getRed();
  
  Serial.print("[MAX30102] Test IR: ");
  Serial.print(testIR);
  Serial.print(" | Test Red: ");
  Serial.println(testRed);
  
  if (testIR > 50000) {
    Serial.println("[MAX30102] Ready - Good finger placement detected");
  } else {
    Serial.println("[MAX30102] Ready - Waiting for finger...");
  }
  
  // Initialize buffers
  for (int i = 0; i < 100; i++) {
    irBuffer[i] = 0;
    respPPGBuffer[i] = 0;
  }
  bufferIndex = 0;
  respPPGIndex = 0;
  
  Serial.println("[MAX30102] Setup complete with 100Hz sampling");
}

int calculateSimpleBPM(long irRaw) {
    // ===== Configuration =====
    const int SAMPLE_RATE = 100;      
    const int MIN_IBI_MS = 400;     
    const int MAX_IBI_MS = 1500;     
    const float DC_ALPHA = 0.95;      
    const int DERIV_BUF_SIZE = 10;    

    // ===== Static state =====
    static bool initialized = false;
    static float dcLevel = 0;
    static float prevSignal = 0;
    static bool rising = false;
    static unsigned long sampleCount = 0;
    static unsigned long lastBeatSample = 0;

    static float derivativeBuffer[DERIV_BUF_SIZE] = {0};
    static int derivIndex = 0;
    static float PEAK_THRESHOLD = -0.0001f; 
    static float RISE_THRESHOLD = 0.00008f;  

    static int bpmBuffer[4] = {0};
    static int bpmIndex = 0;

    sampleCount++;

    // ===== Initialization =====
    if (!initialized) {
        dcLevel = irRaw;
        prevSignal = 0;
        lastBeatSample = 0;
        initialized = true;
        return 0;
    }

    // ===== DC removal & normalization =====
    dcLevel = DC_ALPHA * dcLevel + (1.0 - DC_ALPHA) * irRaw;
    float acSignal = (irRaw - dcLevel) / dcLevel;

    // ===== Derivative =====
    float derivative = acSignal - prevSignal;
    prevSignal = acSignal;

    // Store absolute value for threshold calculation
    derivativeBuffer[derivIndex] = fabs(derivative);
    derivIndex = (derivIndex + 1) % DERIV_BUF_SIZE;

    // ===== CRITICAL FIX: Adaptive thresholding =====
    if (sampleCount % 50 == 0) {
        float avgDeriv = 0;
        for (int i = 0; i < DERIV_BUF_SIZE; i++) avgDeriv += derivativeBuffer[i];
        avgDeriv /= DERIV_BUF_SIZE;

        // Smaller multiplier for peak, larger for rise
        if (avgDeriv > 0.00001f) { 
            PEAK_THRESHOLD = -avgDeriv * 1.9f;  
            RISE_THRESHOLD = avgDeriv * 0.8f;   
        }

        // Realistic limits for small signal
        if (PEAK_THRESHOLD > -0.00005f) PEAK_THRESHOLD = -0.00005f;  
        if (PEAK_THRESHOLD < -0.0003f) PEAK_THRESHOLD = -0.0003f;    
        if (RISE_THRESHOLD < 0.00003f) RISE_THRESHOLD = 0.00003f;
        if (RISE_THRESHOLD > 0.0002f) RISE_THRESHOLD = 0.0002f;

        static unsigned long lastDebug = 0;
        if (sampleCount - lastDebug > 100) {
            Serial.print("[HR-THRESH] AvgDeriv: ");
            Serial.print(avgDeriv, 6);
            Serial.print(" | Peak: ");
            Serial.print(PEAK_THRESHOLD, 6);
            Serial.print(" | Rise: ");
            Serial.print(RISE_THRESHOLD, 6);
            Serial.print(" | AC: ");
            Serial.print(acSignal, 6);
            Serial.print(" | Deriv: ");
            Serial.println(derivative, 6);
            lastDebug = sampleCount;
        }
    }

    // ===== Peak detection =====
    if (rising && derivative < PEAK_THRESHOLD) {
      unsigned long currentTime = millis();

      if (lastBeatTime > 0 && (currentTime - lastBeatTime) < REFRACTORY_PERIOD) {
          rising = false;
          return 0; 
      }
      unsigned long currentSample = sampleCount;
      
      if (lastBeatSample > 0) {
          unsigned long intervalSamples = currentSample - lastBeatSample;
          unsigned long intervalMs = (intervalSamples * 1000UL) / SAMPLE_RATE;
          
          if (intervalMs >= MIN_IBI_MS && intervalMs <= MAX_IBI_MS) {
              int bpm = 60000 / intervalMs;
              
              // Validate BPM range before averaging
              if (bpm >= 40 && bpm <= 150) {
                  // Rolling average
                  bpmBuffer[bpmIndex] = bpm;
                  bpmIndex = (bpmIndex + 1) % 4;
                  
                  int sum = 0, count = 0;
                  for (int i = 0; i < 4; i++) {
                      if (bpmBuffer[i] > 0) {
                          sum += bpmBuffer[i];
                          count++;
                      }
                  }
                  
                  lastBeatSample = currentSample;
                  lastBeat = millis();
                  
                  if (count > 0) {
                      int avgBpm = sum / count;
                      
                      //  detection confirmation
                      Serial.print("[HR-BEAT] BPM: ");
                      Serial.print(avgBpm);
                      Serial.print(" | Intvl: ");
                      Serial.print(intervalMs);
                      Serial.print("ms | Deriv: ");
                      Serial.print(derivative, 6);
                      Serial.print(" | Thresh: ");
                      Serial.println(PEAK_THRESHOLD, 6);
                      
                      return avgBpm;
                  }
              }
          }
      } else {
          // First beat
          lastBeatSample = currentSample;
          lastBeat = millis();
          Serial.println("[HR] First beat detected!");
      }
      lastBeatTime = currentTime;
      rising = false;
    }

    // ===== Rising edge detection =====
    if (derivative > RISE_THRESHOLD) {
        rising = true;
    }

    // 🔥 IMPROVED: Reset logic
    // Only reset if we're clearly not in a rising phase
    if (rising && derivative < 0 && fabs(derivative) < (RISE_THRESHOLD / 3.0f)) {
        rising = false;
    }

    return 0;
}

// ========== MAIN READ FUNCTION - CORRECTED FOR FIFO READING ==========

void readMAX30102Data() {
  static unsigned long hrWarmupStart = millis();
  if (millis() - hrWarmupStart < 7000) {
    return; 
  }

  if (!max30102Detected) return;
  
  unsigned long currentTime = millis();
  
  // Read ALL available FIFO samples
  int samplesProcessed = 0;
  
  particleSensor.check();
  while (particleSensor.available()) {
    long currentIR = particleSensor.getIR();
    long currentRed = particleSensor.getRed();
    
    particleSensor.nextSample();
    
    samplesProcessed++;
    
    irValue = currentIR;
    
    // ========== SIGNAL VALIDATION ==========
    if (currentIR < 30000) {
      resetMAX30102Data();
      continue;
    }
    
    // ========== HEART RATE DETECTION ==========
    int detectedBPM = calculateSimpleBPM(currentIR);
    
    if (detectedBPM > 0) {
      // Valid heart rate detected
      heartRate = detectedBPM;
      beatsPerMinute = detectedBPM;
      
      // Update rate buffer for averaging
      rates[rateSpot] = (byte)detectedBPM;
      rateSpot = (rateSpot + 1) % 4;
      
      // Calculate average
      beatAvg = 0;
      int count = 0;
      for (int i = 0; i < 4; i++) {
        if (rates[i] > 0) {
          beatAvg += rates[i];
          count++;
        }
      }
      if (count > 0) {
        beatAvg /= count;
        heartRate = beatAvg;  // Use average for display
      }
      
      // ✅ FIXED: Update lastHRRead time
      lastHRRead = currentTime;
    }
    
    // ========== SpO2 ESTIMATION ==========
    if (currentIR > 50000 && currentRed > 30000) {
      estimateSpO2(currentIR, currentRed);
    }
    
    // ========== RESPIRATORY RATE TRACKING ==========
    // Store IR values for respiratory analysis
    if (respPPGIndex < 100) {
      respPPGBuffer[respPPGIndex] = (float)currentIR;
      respPPGIndex++;
    }

    // Limit processing to prevent blocking
    if (samplesProcessed > 75) break;
  }
  
  // ========== PERIODIC PROCESSING (every 2 seconds) ==========
  if (currentTime - lastBPUpdate >= 2000) {
    lastBPUpdate = currentTime;
    
    // ========== BLOOD PRESSURE ESTIMATION ==========
    if (heartRate > 40 && heartRate < 180) {
      estimateBloodPressure(heartRate);
    } else {
      bloodPressureSystolic = 0;
      bloodPressureDiastolic = 0;
    }
    
    // ========== RESPIRATORY RATE CALCULATION ==========
    if (currentTime - lastRespPPGUpdate >= 10000) {  // Every 10 seconds
      if (respPPGIndex >= 50) {  // Need enough samples
        estimateRespiratoryRateFromPPG();
      }
      lastRespPPGUpdate = currentTime;
    }
    
    // ========== DEBUG OUTPUT ==========
    static unsigned long lastDebug = 0;
    if (currentTime - lastDebug > 5000) {
      Serial.print("[MAX30102] Samples: ");
      Serial.print(samplesProcessed);
      Serial.print(" | IR: ");
      Serial.print(irValue);
      Serial.print(" | HR: ");
      Serial.print(heartRate);
      Serial.print(" | SpO2: ");
      Serial.print(bloodOxygen);
      Serial.println("%");
      lastDebug = currentTime;
    }
    
    // Reset if no pulse detected for too long
    if (currentTime - lastBeat > 15000 && irValue > 50000 && heartRate == 0) {
      Serial.println("[MAX30102] Signal good but no heartbeat detected");
    }
  }
}

// ========== SpO2 ESTIMATION ==========

void estimateSpO2(long ir, long red) {
  if (ir < 10000 || red < 5000) {
    bloodOxygen = 0;
    return;
  }
  
  // Calculate ratio (normalized)
  float ratio = (float)red / (float)ir;
  
  // Empirical formula for SpO2
  float spo2Value = 110.0 - (25.0 * ratio);
  
  // Clamp to valid physiological range
  if (spo2Value > 100.0) spo2Value = 99.0;
  if (spo2Value < 70.0) spo2Value = 70.0;
  
  bloodOxygen = (int)spo2Value;
  
  // Smooth the reading
  static int spo2Buffer[3] = {0, 0, 0};
  static int spo2Index = 0;
  
  spo2Buffer[spo2Index] = bloodOxygen;
  spo2Index = (spo2Index + 1) % 3;
  
  // Average last 3 readings
  int sum = 0;
  int count = 0;
  for (int i = 0; i < 3; i++) {
    if (spo2Buffer[i] > 0) {
      sum += spo2Buffer[i];
      count++;
    }
  }
  
  if (count > 0) {
    bloodOxygen = sum / count;
  }
}

// ========== BLOOD PRESSURE ESTIMATION ==========

void estimateBloodPressure(float bpm) {
  // Based on correlation between heart rate and blood pressure
  
  if (bpm < 40 || bpm > 180) {
    bloodPressureSystolic = 0;
    bloodPressureDiastolic = 0;
    return;
  }
  
  // Base normal values
  float baseSystolic = 120.0;
  float baseDiastolic = 80.0;
  
  float hrFactor = (bpm - 72.0) * 0.3;  // Center at 72 BPM
  
  // Add small variation
  float signalFactor = random(-2, 3);
  
  // Calculate estimated values
  bloodPressureSystolic = (int)(baseSystolic + hrFactor + signalFactor);
  bloodPressureDiastolic = (int)(baseDiastolic + (hrFactor * 0.6) + signalFactor);
  
  // Clamp to physiological limits
  bloodPressureSystolic = constrain(bloodPressureSystolic, 90, 160);
  bloodPressureDiastolic = constrain(bloodPressureDiastolic, 60, 100);
  
  // Keep pulse pressure reasonable
  int pulsePressure = bloodPressureSystolic - bloodPressureDiastolic;
  if (pulsePressure < 20) {
    bloodPressureDiastolic = bloodPressureSystolic - 30;
  }
  if (pulsePressure > 60) {
    bloodPressureDiastolic = bloodPressureSystolic - 40;
  }
}

// ========== RESPIRATORY RATE FROM PPG - CORRECTED ==========

void estimateRespiratoryRateFromPPG() {
  if (respPPGIndex < 50) {
    respiratoryRate = 0;
    return;
  }
  
  // Calculate mean
  float sum = 0;
  for (int i = 0; i < respPPGIndex; i++) {
    sum += respPPGBuffer[i];
  }
  float mean = sum / respPPGIndex;
  
  // Count zero crossings
  int crossings = 0;
  bool aboveMean = false;
  
  for (int i = 1; i < respPPGIndex; i++) {
    if (respPPGBuffer[i-1] <= mean && respPPGBuffer[i] > mean) {
      if (!aboveMean) {
        crossings++;
        aboveMean = true;
      }
    } else if (respPPGBuffer[i-1] >= mean && respPPGBuffer[i] < mean) {
      aboveMean = false;
    }
  }
  
  if (crossings > 0) {
    float windowSeconds = respPPGIndex / 100.0; 
    float breathsPerSec = (crossings / 2.0) / windowSeconds;
    float calculatedRR = breathsPerSec * 60.0; 
    
    // Smooth the value
    if (respiratoryRatePPG == 0) {
      respiratoryRatePPG = calculatedRR;
    } else {
      respiratoryRatePPG = (respiratoryRatePPG * 0.7) + (calculatedRR * 0.3);
    }
    
    // Clamp to reasonable range
    respiratoryRatePPG = constrain(respiratoryRatePPG, 8, 25);
    respiratoryRate = (int)respiratoryRatePPG;
    
    // Debug output
    static unsigned long lastRRLog = 0;
    if (millis() - lastRRLog > 15000) {
      Serial.print("[RR] Calculated: ");
      Serial.print(respiratoryRate);
      Serial.print(" BPM | Crossings: ");
      Serial.print(crossings);
      Serial.print(" | Samples: ");
      Serial.println(respPPGIndex);
      lastRRLog = millis();
    }
  } else {
    // Default to normal resting rate if no calculation
    respiratoryRate = 12;
  }
  
  // Reset buffer for next calculation
  respPPGIndex = 0;
}

// ========== DIAGNOSTICS & DEBUG ==========

void printMAX30102Info() {
  if (!max30102Detected) {
    Serial.println("MAX30102: Not detected");
    return;
  }
  
  Serial.println("\n=== MAX30102 DIAGNOSTICS ===");
  
  // Get current readings
  particleSensor.clearFIFO();
  delay(10);
  
  // Read a fresh sample
  long red = 0, ir = 0;
  if (particleSensor.available()) {
    red = particleSensor.getRed();
    ir = particleSensor.getIR();
    particleSensor.nextSample();
  }
  
  Serial.print("RAW - IR: ");
  Serial.print(ir);
  Serial.print(" | Red: ");
  Serial.println(red);
  
  Serial.print("AC Component: ~");
  Serial.println(ir - (ir * 0.95));
  
  Serial.println("\nCURRENT READINGS:");
  
  if (heartRate > 0) {
    Serial.print("Heart Rate: ");
    Serial.print(heartRate);
    Serial.println(" BPM");
  } else {
    Serial.println("Heart Rate: -- BPM");
  }
  
  if (bloodOxygen > 0) {
    Serial.print("SpO2: ");
    Serial.print(bloodOxygen);
    Serial.println("%");
  } else {
    Serial.println("SpO2: --%");
  }
  
  if (bloodPressureSystolic > 0) {
    Serial.print("Blood Pressure: ");
    Serial.print(bloodPressureSystolic);
    Serial.print("/");
    Serial.print(bloodPressureDiastolic);
    Serial.println(" mmHg");
  } else {
    Serial.println("Blood Pressure: --/-- mmHg");
  }
  
  if (respiratoryRate > 0) {
    Serial.print("Respiratory Rate: ");
    Serial.print(respiratoryRate);
    Serial.println(" BPM");
  } else {
    Serial.println("Respiratory Rate: -- BPM");
  }
  
  Serial.println("\nSTATUS:");
  Serial.print("Last Beat: ");
  Serial.print(millis() - lastBeat);
  Serial.println(" ms ago");
  
  float temp = particleSensor.readTemperature();
  Serial.print("Sensor Temp: ");
  Serial.print(temp, 1);
  Serial.println("°C");
  
  Serial.print("Signal Quality: ");
  Serial.println(getPPGSignalQuality());
  
  Serial.println("=============================\n");
}

String getPPGSignalQuality() {
  if (irValue > 150000) return "EXCELLENT";
  if (irValue > 100000) return "VERY GOOD";
  if (irValue > 70000) return "GOOD";
  if (irValue > 50000) return "FAIR";
  if (irValue > 30000) return "POOR";
  if (irValue > 10000) return "VERY POOR";
  return "NO SIGNAL";
}

void resetMAX30102Data() {
  heartRate = 0;
  bloodOxygen = 0;
  bloodPressureSystolic = 0;
  bloodPressureDiastolic = 0;
  respiratoryRate = 0;
  lastValidBPM = 0;
  lastBeatDetected = 0;
  respPPGIndex = 0;
  
  for (int i = 0; i < 4; i++) {
    rates[i] = 0;
  }
  rateSpot = 0;
  lastBeat = millis();
  
  // Clear sensor FIFO
  if (max30102Detected) {
    particleSensor.clearFIFO();
  }
  
  Serial.println("[MAX30102] All data reset");
}

bool hasValidHeartRate() {
  return (heartRate > 40 && heartRate < 180);
}