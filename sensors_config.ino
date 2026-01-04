// sensor_globals.ino - ALL GLOBAL VARIABLES DECLARED HERE

// Include only basic headers
#include "bitmaps.h"

// ========== POWER & SYSTEM ==========
bool powerState = false;
bool sosActive = false;
bool showingLogo = false;
PageState currentPage = PAGE_VITALS_1;
unsigned long logoStartTime = 0;
unsigned long pageTimer = 0;
unsigned long lastActivityTime = 0;
unsigned long sosStartTime = 0;

// ========== SENSOR STATUS ==========
bool bmp280Detected = false;
bool mpu6050Detected = false;
bool max30102Detected = false;

// ========== VITAL SIGNS ==========
int heartRate = 78;
int bloodOxygen = 97;
float bodyTemperature = 36.5;
float ambientTemperature = 25.0;
int bloodPressureSystolic = 120;
int bloodPressureDiastolic = 80;
int respiratoryRate = 0;
int sleepQuality = 85;
int stepsToday = 4256;
int batteryLevel = 85;
int signalStrength = 4;

// ========== MOTION TRACKING ==========
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
int stepCount = 0;
unsigned long lastStepTime = 0;

// ========== RESPIRATORY TRACKING ==========
float respBuffer[20];
int respIndex = 0;
unsigned long lastRespUpdate = 0;

// ========== HEART RATE TRACKING ==========
const byte RATE_SIZE = 4;  // This is a constant, not a variable
byte rates[4];  // Changed from rates[RATE_SIZE] to rates[4]
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
long irValue;

// ========== TIMING VARIABLES ==========
unsigned long lastHRRead = 0;
unsigned long lastBPUpdate = 0;
unsigned long lastRRUpdate = 0;
unsigned long lastTempRead = 0;
unsigned long lastMotionRead = 0;
unsigned long lastSensorUpdate = 0;
unsigned long lastMAX30102Read = 0;

// ========== ANIMATION STATE ==========
int heartbeatFrame = 0;
int respFrame = 0;
int walkFrame = 0;
int scrollPos = 0;
unsigned long lastHeartbeatUpdate = 0;
unsigned long lastResUpdate = 0;
unsigned long lastWalkUpdate = 0;
unsigned long lastScroll = 0;

// ========== BUTTON STATE ==========
bool buttonActive = false;
bool longPressActive = false;
unsigned long buttonPressTime = 0;

// ========== SENSOR OBJECTS ==========
// Note: These are declared as extern in other files
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include "heartRate.h"
#include "spo2_algorithm.h"

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
MAX30105 particleSensor;