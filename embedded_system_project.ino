#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

enum PetState { IDLE, NEEDS_PLAY, PLAYING, NEEDS_TALK, TALKING, NEEDS_FEED, FEEDING };
PetState currentState = IDLE;

// MPU6050 setup
Adafruit_MPU6050 mpu;

// Pin definitions
#define MIC_PIN           34    // ADC1 channel for microphone input
#define BATTERY_PIN       35    // ADC1 channel for battery voltage
#define BUZZER_PIN        25    // DAC output for buzzer
#define LED_20            2     // LED for 20% battery level
#define LED_40            4     // LED for 40% battery level
#define LED_60            16    // LED for 60% battery level
#define LED_80            17    // LED for 80% battery level
#define LED_100           5     // LED for 100% battery level

// Battery voltage calibration
#define BATTERY_FULL_VOLTAGE 4.2
#define BATTERY_EMPTY_VOLTAGE 3.0

// Microphone band-pass filter constants
#define MIC_THRESHOLD 500 

// Filter coefficients
float b[5] = {0.00289819, 0.0, -0.00579639, 0.0, 0.00289819}; // Numerator coefficients
float a[5] = {1.0, -3.72372947, 5.24211767, -3.3152209, 0.79743236}; // Denominator coefficients

float x[5] = {0, 0, 0, 0, 0}; // Input samples buffer
float y[5] = {0, 0, 0, 0, 0}; // Output samples buffer

// Pet needs
int needToTalk = 10;
int needToPlay = 10;
int needToFeed = 10;

// Time tracking
unsigned long lastNeedUpdate = 0;
unsigned long lastBeepTime = 0;
const unsigned long needDepleteInterval = 2000;  // 2 seconds
const unsigned long beepInterval = 500;          // Beep every 500ms when warning

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Set pin modes for LEDs
  pinMode(LED_20, OUTPUT);
  pinMode(LED_40, OUTPUT);
  pinMode(LED_60, OUTPUT);
  pinMode(LED_80, OUTPUT);
  pinMode(LED_100, OUTPUT);

  // Clear LEDs initially
  clearBatteryLEDs();
}

void loop() {
  // Monitor and update pet needs
  updatePetNeeds();

  // Check battery status periodically
  int batteryRaw = analogRead(BATTERY_PIN);
  float batteryVoltage = (batteryRaw / 4095.0) * 3.3 * 2; // Assuming a voltage divider circuit
  updateBatteryLEDs(batteryVoltage);

  switch (currentState) {
    case IDLE:
      // Pet is idle, waiting for interaction
      Serial.println("Pet is idle.");
      checkNeeds();
      break;

    case NEEDS_PLAY:
      // Pet needs to be played with, activate MPU6050
      Serial.println("Pet needs to play. Move it!");
      checkMovement();
      break;

    case PLAYING:
      // Pet is being played with
      Serial.println("Playing with pet...");
      playWithPet();
      break;

    case NEEDS_TALK:
      // Pet needs to be talked to, activate microphone
      Serial.println("Pet needs to talk. Say something!");
      listenForVoice();
      break;

    case TALKING:
      // Pet is listening to the voice
      Serial.println("Talking to pet...");
      respondToVoice();
      break;

    case NEEDS_FEED:
      // Pet needs to be fed
      Serial.println("Pet needs to be fed. Feed it!");
      // Placeholder for feeding logic
      // currentState = FEEDING; // Transition to feeding state if implemented
      break;

    case FEEDING:
      // Placeholder for feeding logic
      Serial.println("Feeding the pet...");
      // Reset feed need and return to idle
      needToFeed = 10;
      currentState = IDLE;
      break;
  }

  delay(100);  // Adjust delay as needed
}

void updatePetNeeds() {
  if (millis() - lastNeedUpdate > needDepleteInterval) {
    lastNeedUpdate = millis();
    if (needToTalk > 0) needToTalk--;
    if (needToPlay > 0) needToPlay--;
    if (needToFeed > 0) needToFeed--;
    checkWarnings();
  }
}

void checkWarnings() {
  if (needToTalk <= 3 || needToPlay <= 3 || needToFeed <= 3) {
    if (millis() - lastBeepTime > beepInterval) {
      lastBeepTime = millis();
      tone(BUZZER_PIN, 1000, 200); // Beep as a warning
    }
    if (needToTalk <= 3) {
      Serial.println("Pet needs to talk!");
    }
    if (needToPlay <= 3) {
      Serial.println("Pet needs to play!");
    }
    if (needToFeed <= 3) {
      Serial.println("Pet needs to be fed!");
    }
  }
}

void checkNeeds() {
  if (needToPlay <= 3) {
    currentState = NEEDS_PLAY;
  } else if (needToTalk <= 3) {
    currentState = NEEDS_TALK;
  } else if (needToFeed <= 3) {
    currentState = NEEDS_FEED;
  }
}

void checkMovement() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Simple movement detection
  if (abs(a.acceleration.x) > 1.0 || abs(a.acceleration.y) > 1.0 || abs(a.acceleration.z) > 1.0) {
    Serial.println("Movement detected!");
    needToPlay = 10; // Reset play need
    currentState = PLAYING;
  }
}

void playWithPet() {
  // Generate some success feedback via buzzer
  for (int freq = 1000; freq <= 2000; freq += 250) {
    tone(BUZZER_PIN, freq, 100);
    delay(150);
  }

  // After playing, return to idle or another state
  currentState = IDLE;
}

void listenForVoice() {
  float micInput = analogRead(MIC_PIN);
  float filteredOutput;

  applyBandpassFilter(micInput, filteredOutput);

  if (filteredOutput > MIC_THRESHOLD) {
    Serial.println("Voice detected!");
    needToTalk = 10; // Reset talk need
    currentState = TALKING;
  }
}

void respondToVoice() {
  // Play a sound or perform some action
  tone(BUZZER_PIN, 1500, 200);
  delay(500);
  currentState = IDLE;  // Go back to idle after talking
}

void applyBandpassFilter(float input, float &output) {
  // Shift the input samples buffer
  for (int i = 4; i > 0; i--) {
    x[i] = x[i-1];
    y[i] = y[i-1];
  }
  x[0] = input;

  // Apply the filter
  output = b[0] * x[0] + b[1] * x[1] + b[2] * x[2] + b[3] * x[3] + b[4] * x[4]
           - a[1] * y[0] - a[2] * y[1] - a[3] * y[2] - a[4] * y[3];

  y[0] = output;
}

void clearBatteryLEDs() {
  digitalWrite(LED_20, LOW);
  digitalWrite(LED_40, LOW);
  digitalWrite(LED_60, LOW);
  digitalWrite(LED_80, LOW);
  digitalWrite(LED_100, LOW);
}

void updateBatteryLEDs(float voltage) {
  clearBatteryLEDs();

  if (voltage >= BATTERY_FULL_VOLTAGE) {
    digitalWrite(LED_100, HIGH);
  } else if (voltage >= BATTERY_FULL_VOLTAGE - 0.2) {
    digitalWrite(LED_80, HIGH);
  } else if (voltage >= BATTERY_FULL_VOLTAGE - 0.4) {
    digitalWrite(LED_60, HIGH);
  } else if (voltage >= BATTERY_FULL_VOLTAGE - 0.6) {
    digitalWrite(LED_40, HIGH);
  } else if (voltage >= BATTERY_FULL_VOLTAGE - 0.8) {
    digitalWrite(LED_20, HIGH);
  }
}
