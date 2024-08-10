#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

// Wi-Fi and MQTT configuration
const char* ssid = "Kaljakoppakuoriainen";
const char* password = "HomoPaavi666";
const char* mqtt_server = "192.168.1.153"; // Replace with your MQTT broker IP

WiFiClient espClient;
PubSubClient client(espClient);

// Define FSM states
enum GameState { IDLE, PLAYING, FEEDING, TALKING, END_GAME };
GameState currentState = IDLE;

// Sensor setup
Adafruit_MPU6050 mpu;
bool mpuInitialized = false;  // To check if MPU6050 is initialized

// Pin definitions
#define MIC_PIN           9    // ADC1 channel for microphone input
#define BATTERY_PIN       4    // ADC1 channel for battery voltage
#define BUZZER_PIN        40   // DAC output for buzzer
#define LED_20            15   // LED for 20% battery level
#define LED_40            16   // LED for 40% battery level
#define LED_60            17   // LED for 60% battery level
#define LED_80            18   // LED for 80% battery level
#define LED_100           14   // LED for 100% battery level
#define DEBUG_LED1        36   // LED1 for debugging
#define DEBUG_LED2        35   // LED2 for debugging
#define MPU_SCL           39   // MPU SCL
#define MPU_SDA           38   // MPU SDA
#define MPU_INT           37   // MPU INT

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

void setup() {
  Serial.begin(115200);

  // Wait for the Serial Monitor to open
  while (!Serial) {
    ; // Wait until the serial connection is established
  }
  
  Serial.println("ESP32 is ready!");

  // Initialize Wi-Fi
  setup_wifi();

  // Initialize MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Connect to MQTT broker
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT broker");
      client.subscribe("esp32/game/command");
    } else {
      delay(5000);
    }
  }

  // Initialize MPU6050
  for (int i = 0; i < 5; i++) { // Retry up to 5 times
    if (mpu.begin()) {
      Serial.println("MPU6050 initialized successfully");
      mpuInitialized = true;
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      break;
    } else {
      Serial.println("Failed to find MPU6050 chip, retrying...");
      delay(2000); // Wait 2 seconds before retrying
    }
  }

  if (!mpuInitialized) {
    Serial.println("MPU6050 not found. Continuing without sensor.");
  }

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
  if (!client.connected()) {
    // Reconnect if connection is lost
    while (!client.connected()) {
      client.connect("ESP32Client");
    }
    client.subscribe("esp32/game/command");
  }
  client.loop();

  // Handle the current state
  switch (currentState) {
    case IDLE:
      // Do nothing, waiting for commands from the backend
      break;

    case PLAYING:
      Serial.println("Playing with pet...");
      delay(1000);
      checkMovementAndPlay(); // Combined movement check and play action
      currentState = IDLE; // Go back to idle after playing
      break;

    case FEEDING:
      Serial.println("Feeding the pet...");
      delay(1000);
      checkFeedingAction(); // Check the feeding action
      currentState = IDLE; // Go back to idle after feeding
      break;

    case TALKING:
      Serial.println("Talking to pet...");
      delay(1000);
      listenForVoice();
      currentState = IDLE; // Go back to idle after talking
      break;

    case END_GAME:
      playDyingSong();
      delay(5000);  // Delay to allow the dying song to play
      Serial.println("Game Over. Pet has died.");
      esp_deep_sleep_start();  // Put the ESP32 into deep sleep
      break;
  }

  // Check battery status periodically
  int batteryRaw = analogRead(BATTERY_PIN);
  float batteryVoltage = (batteryRaw / 4095.0) * 3.3 * 2; // Assuming a voltage divider circuit
  updateBatteryLEDs(batteryVoltage);

  // Echo serial input (for testing or additional control)
  if (Serial.available() > 0) {
    String incomingData = Serial.readString();
    Serial.print("Received: ");
    Serial.println(incomingData);
  }
}

// Wi-Fi setup function
void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

// MQTT callback function
void callback(char* topic, byte* message, unsigned int length) {
  String receivedMessage;

  for (int i = 0; i < length; i++) {
    receivedMessage += (char)message[i];
  }

  if (receivedMessage.startsWith("alert_")) {
    Serial.println("Alert received!");
    playAlertBeep();
  } else if (receivedMessage == "play") {
    Serial.println("Command received: Play");
    currentState = PLAYING;
  } else if (receivedMessage == "talk") {
    Serial.println("Command received: Talk");
    currentState = TALKING;
  } else if (receivedMessage == "feed") {
    Serial.println("Command received: Feed");
    currentState = FEEDING;
  } else if (receivedMessage == "stop_game") {
    Serial.println("Command received: Stop Game");
    currentState = END_GAME;
  } else {
    client.publish("esp32/game/response", "ACTION FAILED");
  }
}

// Play an alert beep
void playAlertBeep() {
  tone(BUZZER_PIN, 1000, 500);  // 1000 Hz for 500 ms
  delay(500);
}

// Combined movement check and play function
void checkMovementAndPlay() {
  if (!mpuInitialized) {
    Serial.println("MPU6050 not initialized. Skipping movement check.");
    client.publish("esp32/game/response", "ACTION FAILED");
    return; // Exit if the MPU6050 is not initialized
  }

  const int shakeDuration = 3000; // Required shake time in milliseconds
  const int checkInterval = 100;  // Interval to check for movement (in ms)
  unsigned long startTime = millis();
  unsigned long currentTime;
  bool movementDetected = false;

  Serial.println("Start shaking the device...");

  while ((currentTime = millis()) - startTime < shakeDuration) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Simple movement detection logic (can be expanded)
    if (abs(a.acceleration.x) > 1.0 || abs(a.acceleration.y) > 1.0 || abs(a.acceleration.z) > 1.0) {
      movementDetected = true;
      Serial.println("Movement detected!");
      break; // Exit the loop as soon as movement is detected
    }

    delay(checkInterval); // Small delay to avoid overwhelming the sensor
  }

  if (movementDetected) {
    // If shaking was detected, play the success beep
    playSuccessBeep();
    client.publish("esp32/game/response", "ACTION OK");
  } else {
    client.publish("esp32/game/response", "ACTION FAILED");
  }

  delay(1000); // Delay after completing the action
}


// Play a success beep
void playSuccessBeep() {
  for (int freq = 1000; freq <= 2000; freq += 250) {
    tone(BUZZER_PIN, freq, 100);
    delay(150);
  }
}

// Play a dying song
void playDyingSong() {
  int melody[] = {262, 247, 233, 220, 196, 185, 174, 165};
  int noteDurations[] = {500, 500, 500, 500, 500, 500, 500, 500};

  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);
    delay(noteDuration * 1.3);  // to distinguish the notes
  }
}

// Check the feeding action by detecting a tilt
void checkFeedingAction() {
  if (!mpuInitialized) {
    Serial.println("MPU6050 not initialized. Skipping feeding check.");
    client.publish("esp32/game/response", "ACTION FAILED");
    return; // Exit if the MPU6050 is not initialized
  }

  const int waitDuration = 3000; // 3 seconds waiting period
  unsigned long startTime = millis();
  unsigned long currentTime;
  bool tiltDetected = false;

  Serial.println("Waiting for tilt...");

  // Wait for 3 seconds to detect a tilt
  while ((currentTime = millis()) - startTime < waitDuration) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Assuming horizontal is z > 9.0 m/s² (gravity)
    // Check if the device is tilted to at least 90 degrees
    if (a.acceleration.z < 0.0 && abs(a.acceleration.x) > 5.0) {
      tiltDetected = true;
      break;
    }

    delay(100); // Small delay to avoid overwhelming the sensor
  }

  // Check the result of the tilt detection
  if (tiltDetected) {
    Serial.println("Feeding action successful!");
    playSuccessBeep();
    client.publish("esp32/game/response", "ACTION OK");
  } else {
    Serial.println("Feeding action failed. No tilt detected within the time limit.");
    client.publish("esp32/game/response", "ACTION FAILED");
  }

  delay(1000); // Delay after completing the action
}

// Listen for voice using the microphone input for 3 seconds
void listenForVoice() {
  const int listenDuration = 3000; // Listen for 3 seconds
  unsigned long startTime = millis();
  unsigned long currentTime;
  bool voiceDetected = false;

  while ((currentTime = millis()) - startTime < listenDuration) {
    float micInput = analogRead(MIC_PIN);
    float filteredOutput;

    applyBandpassFilter(micInput, filteredOutput);

    if (filteredOutput > MIC_THRESHOLD) {
      voiceDetected = true;
      Serial.println("Voice detected!");
      break; // once voice is recognized, end the loop
    }

    delay(100); // Small delay to avoid overwhelming the sensor
  }

  if (voiceDetected) {
    Serial.println("Voice input successful!");
    playSuccessBeep();  // Play a success beep
    client.publish("esp32/game/response", "ACTION OK");
  } else {
    Serial.println("No voice detected in the given time.");
    client.publish("esp32/game/response", "ACTION FAILED");
  }
}

// Apply band-pass filter to the microphone input
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

// Clear battery status LEDs
void clearBatteryLEDs() {
  digitalWrite(LED_20, LOW);
  digitalWrite(LED_40, LOW);
  digitalWrite(LED_60, LOW);
  digitalWrite(LED_80, LOW);
  digitalWrite(LED_100, LOW);
}

// Update battery status LEDs based on voltage
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