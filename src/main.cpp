#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// === Color Sensor Setup ===
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

struct ColorReference {
  const char* name;
  float r, g, b;
};

ColorReference knownColors[] = {
  {"Brown", 90.9, 92.3, 53.6},
  {"White", 80.4, 96.3, 60.0},
  {"Clear", 75.9, 99.5, 61.2}
};

const int numColors = sizeof(knownColors) / sizeof(ColorReference);
const char* expectedSequence[] = {"Brown", "White", "Clear"};
const int sequenceLength = 2;  // Adjusted for quicker test
int sequenceState = 0;

float rgbDistance(float r1, float g1, float b1, float r2, float g2, float b2) {
  return sqrt(pow(r1 - r2, 2) + pow(g1 - g2, 2) + pow(b1 - b2, 2));
}

// === Motor Setup ===
const int motorL_Pin1 = 17, motorL_Pin2 = 16;
const int motorR_Pin1 = 26, motorR_Pin2 = 27;

String motorSide = "left";
int fixedPWM = 70;
bool systemRunning = true;

// === PWM Setup ===
void setupPWM() {
  ledcAttachPin(motorL_Pin1, 0);
  ledcAttachPin(motorL_Pin2, 1);
  ledcAttachPin(motorR_Pin1, 2);
  ledcAttachPin(motorR_Pin2, 3);
  for (int i = 0; i < 4; i++) {
    ledcSetup(i, 1000, 8); // 1 kHz, 8-bit
  }
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    ledcWrite(i, 0);
  }
}

void driveMotor(int pwm, int pin1, int pin2) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    ledcWrite(pin1, pwm);
    ledcWrite(pin2, 0);
  } else {
    ledcWrite(pin1, 0);
    ledcWrite(pin2, -pwm);
  }
}

void driveCurrentMotor() {
  if (!systemRunning) return;
  if (motorSide == "left") driveMotor(fixedPWM, 0, 1);
  else                     driveMotor(fixedPWM, 2, 3);
}

void toggleDirection() {
  stopAllMotors();
  systemRunning = false;
  delay(300);  // pause before switching
  motorSide = (motorSide == "left") ? "right" : "left";
  systemRunning = true;
  Serial.printf("🔁 Switched to %s motor\n", motorSide.c_str());
}

// === Serial Input ===
void handleSerialInput() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.length() == 0) {
    stopAllMotors();
    systemRunning = false;
    Serial.println("[Paused] Format: pwm=100 side=left/right");
    return;
  }

  int idx;
  if ((idx = input.indexOf("pwm=")) != -1) fixedPWM = input.substring(idx + 4).toInt();
  if ((idx = input.indexOf("side=")) != -1) {
    motorSide = input.substring(idx + 5);
    motorSide.trim();
  }

  systemRunning = true;
  Serial.printf("▶️ Running | Motor: %s | PWM: %d\n", motorSide.c_str(), fixedPWM);
  driveCurrentMotor();
}

// === Color Sensor Logic ===
void checkTapeEndColorSequence() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c == 0) c = 1;
  float rn = (r * 1.0 / c) * 255.0;
  float gn = (g * 1.0 / c) * 255.0;
  float bn = (b * 1.0 / c) * 255.0;

  const char* bestMatch = "Unknown";
  float minDist = 1e6;
  for (int i = 0; i < numColors; i++) {
    float d = rgbDistance(rn, gn, bn, knownColors[i].r, knownColors[i].g, knownColors[i].b);
    if (d < minDist) {
      minDist = d;
      bestMatch = knownColors[i].name;
    }
  }

  if (strcmp(bestMatch, expectedSequence[sequenceState]) == 0) {
    sequenceState++;
    Serial.printf("→ Sequence progress: %d/%d\n", sequenceState, sequenceLength);
    delay(100);  // ✅ debounce added back
  } else if (sequenceState > 0 && strcmp(bestMatch, expectedSequence[sequenceState - 1]) == 0) {
    // Still on same color
  } else {
    sequenceState = 0;
  }

  if (sequenceState == sequenceLength) {
    Serial.println("🎉 TAPE ENDED!");
    sequenceState = 0;
    toggleDirection();
  }
}

// === Setup ===
void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!tcs.begin()) {
    Serial.println("TCS34725 not found");
    while (1);
  }
  Serial.println("✅ Color sensor initialized.");

  setupPWM();
  Serial.println("System ready. Use: pwm=... side=left/right");
}

// === Loop ===
void loop() {
  handleSerialInput();
  if (systemRunning) {
    driveCurrentMotor();
    checkTapeEndColorSequence();
  }
}
// pwm=0 side=left
// pwm=150 side=left
// pwm=0 side=right
// pwm=100 side=right