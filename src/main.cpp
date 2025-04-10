#include <Arduino.h>
#include <PID_v1.h>
#include <Adafruit_INA260.h>

// === Motor PWM Pins ===
const int motorA_Pin1 = 17, motorA_Pin2 = 16;  // Left motor (A)
const int motorB_Pin1 = 26, motorB_Pin2 = 27;  // Right motor (B)

// === Encoder Pins ===
const int encoderPinA = 18;  // Motor A
const int encoderPinB = 19;  // Motor B
volatile long encoderTicksA = 0;
volatile long encoderTicksB = 0;

// === Encoder Setup ===
const int MOTOR_PPR = 7;
const int GEAR_RATIO = 20;
const int PULSES_PER_REV = MOTOR_PPR * GEAR_RATIO;

// === PID for Master ===
double rpmTarget = 40.0;
double rpmMeasured = 0;
double pwmMaster = 0;
double Kp = 0.2, Ki = 0.07, Kd = 0.01;
PID speedPID(&rpmMeasured, &pwmMaster, &rpmTarget, Kp, Ki, Kd, DIRECT);

// === Tension PI Controller ===
double currentMaster = 0;
double currentSlave = 0;
double pwmSlave = 0;
double Kp_T = 0.5, Ki_T = 0.05;
double tensionRatio = 0.8;
double tensionError = 0;
double tensionIntegral = 0;

// === INA260 Sensors ===
Adafruit_INA260 sensorA;  // Motor A (left)
Adafruit_INA260 sensorB;  // Motor B (right)

// === Control State ===
String masterSide = "left";  // "left" or "right"

// === Timing ===
const int loopInterval = 150;
unsigned long lastLoopTime = 0;
bool systemRunning = true;

// === Encoder ISRs ===
void IRAM_ATTR encoderISRA() { encoderTicksA++; }
void IRAM_ATTR encoderISRB() { encoderTicksB++; }

// === Motor PWM Setup ===
void setupPWM() {
  ledcAttachPin(motorA_Pin1, 0);
  ledcAttachPin(motorA_Pin2, 1);
  ledcAttachPin(motorB_Pin1, 2);
  ledcAttachPin(motorB_Pin2, 3);
  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);
  ledcSetup(2, 1000, 8);
  ledcSetup(3, 1000, 8);
}

void driveMotor(double pwm, int pin1, int pin2) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    ledcWrite(pin1, pwm);
    ledcWrite(pin2, 0);
  } else {
    ledcWrite(pin1, 0);
    ledcWrite(pin2, -pwm);
  }
}

void stopAllMotors() {
  ledcWrite(0, 0); ledcWrite(1, 0);
  ledcWrite(2, 0); ledcWrite(3, 0);
}

// === Encoder RPM Update ===
void updateEncoderRPM() {
  long ticks;
  if (masterSide == "left") {
    noInterrupts(); ticks = encoderTicksA; encoderTicksA = 0; interrupts();
  } else {
    noInterrupts(); ticks = encoderTicksB; encoderTicksB = 0; interrupts();
  }

  double revs = ticks / (double)PULSES_PER_REV;
  rpmMeasured = revs * (60000.0 / loopInterval);
}

void resetControllers() {
  speedPID.SetMode(MANUAL);
  pwmMaster = 0;
  rpmMeasured = 0;

  tensionError = 0;
  tensionIntegral = 0;
  pwmSlave = 0;

  encoderTicksA = 0;
  encoderTicksB = 0;

  speedPID.SetTunings(Kp, Ki, Kd);
  speedPID.SetMode(AUTOMATIC);
}

// === Serial Tuning Input ===
void handleSerialInput() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.length() == 0) {
    systemRunning = false;
    stopAllMotors();
    Serial.println("[Paused] Format: Kp=... Ki=... Kd=... set=... Kp_T=... Ki_T=... Tr=... master=left/right");
    return;
  }

  double newKp = Kp, newKi = Ki, newKd = Kd, newSet = rpmTarget;
  double newKp_T = Kp_T, newKi_T = Ki_T, newTr = tensionRatio;
  String newMaster = masterSide;

  int idx;
  if ((idx = input.indexOf("Kp=")) != -1) newKp = input.substring(idx + 3).toFloat();
  if ((idx = input.indexOf("Ki=")) != -1) newKi = input.substring(idx + 3).toFloat();
  if ((idx = input.indexOf("Kd=")) != -1) newKd = input.substring(idx + 3).toFloat();
  if ((idx = input.indexOf("set=")) != -1) newSet = input.substring(idx + 4).toFloat();
  if ((idx = input.indexOf("Kp_T=")) != -1) newKp_T = input.substring(idx + 5).toFloat();
  if ((idx = input.indexOf("Ki_T=")) != -1) newKi_T = input.substring(idx + 5).toFloat();
  if ((idx = input.indexOf("Tr=")) != -1) newTr = input.substring(idx + 3).toFloat();
  if ((idx = input.indexOf("master=")) != -1) {
    String requested = input.substring(idx + 7);
    String newMasterSide = (requested == "left") ? "right" : "left";
    if (newMasterSide != masterSide) {
      masterSide = newMasterSide;
      resetControllers();
    }
  }

  Kp = newKp; Ki = newKi; Kd = newKd; rpmTarget = newSet;
  Kp_T = newKp_T; Ki_T = newKi_T; tensionRatio = newTr;
  speedPID.SetTunings(Kp, Ki, Kd);
  systemRunning = true;

  Serial.printf("Running | Master: %s | set=%.1f | Kp_T=%.2f Tr=%.2f\n",
                masterSide.c_str(), rpmTarget, Kp_T, tensionRatio);
}

// === Tension Control ===
void updateSlaveMotorPI() {
  if (masterSide == "left") {
    currentMaster = sensorA.readCurrent();
    currentSlave  = sensorB.readCurrent();
  } else {
    currentMaster = sensorB.readCurrent();
    currentSlave  = sensorA.readCurrent();
  }

  double targetCurrent = tensionRatio * currentMaster;
  tensionError = targetCurrent - currentSlave;
  tensionIntegral += tensionError * (loopInterval / 1000.0);
  tensionIntegral = constrain(tensionIntegral, -100, 100);

  pwmSlave = (Kp_T * tensionError) + (Ki_T * tensionIntegral);

  if (masterSide == "left") {
    driveMotor(pwmSlave, 2, 3);  // drive motor B
  } else {
    driveMotor(pwmSlave, 0, 1);  // drive motor A
  }
}

// === Debug ===
void logStatus() {
  Serial.printf("RPM: %.1f | PWM Master: %.1f | PWM Slave: %.1f | ISlave: %.1f | IMaster: %.1f\n",
    rpmMeasured, pwmMaster, pwmSlave, currentSlave, currentMaster);
}

// === Setup ===
void setup() {
  Serial.begin(9600);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISRB, RISING);

  setupPWM();

  if (!sensorA.begin()) {
    Serial.println("INA260 A not found!");
    while (1);
  }

  if (!sensorB.begin(0x41)) {
    Serial.println("INA260 B not found at 0x41!");
    while (1);
  }

  sensorA.setAveragingCount(INA260_COUNT_128);
  sensorB.setAveragingCount(INA260_COUNT_128);

  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(loopInterval);
  speedPID.SetOutputLimits(-255, 255);

  Serial.println("System ready. Example:");
  Serial.println("Kp=0.25 Ki=0.07 Kd=0.01 set=40 Kp_T=1.0 Ki_T=0.05 Tr=1.0 master=left");
}

// === Loop ===
void loop() {
  handleSerialInput();
  if (!systemRunning) return;

  unsigned long now = millis();
  if (now - lastLoopTime >= loopInterval) {
    lastLoopTime = now;

    updateEncoderRPM();
    speedPID.Compute();

    if (masterSide == "left") {
      driveMotor(pwmMaster, 0, 1);  // Motor A
    } else {
      driveMotor(pwmMaster, 2, 3);  // Motor B
    }

    updateSlaveMotorPI();
    logStatus();
  }
}

// Kp=0.2 Ki=0.07 Kd=0.01 set=40 Kp_T=0.5 Ki_T=0.05 Tr=0.8 master=left
// Kp=0.2 Ki=0.07 Kd=0.01 set=40 Kp_T=0.5 Ki_T=0.05 Tr=0.8 master=right