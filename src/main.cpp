#include <Arduino.h>
#include <PID_v1.h>

// === MOTOR SETUP ===
const int motorIn1 = 17;
const int motorIn2 = 16;

// === ENCODER SETUP ===
const int encoderPinA = 18;
const int MOTOR_PPR = 7;
const int GEAR_RATIO = 20;
const int PULSES_PER_REV = MOTOR_PPR * GEAR_RATIO; // = 140
volatile long encoderTicks = 0;

// === PID VARIABLES ===
double targetRPM = 60.0;
double currentRPM = 0;
double pwmOutput = 0;
double Kp = 0.2, Ki = 0.07, Kd = 0.01;
PID motorPID(&currentRPM, &pwmOutput, &targetRPM, Kp, Ki, Kd, DIRECT);

// === TIMING ===
const int interval = 150; // ms
unsigned long lastTime = 0;

// === CONTROL STATE ===
bool systemRunning = true;

// === ISR ===
void IRAM_ATTR encoderISR() {
  encoderTicks++;
}

// === PWM SETUP ===
void setupPWM() {
  ledcAttachPin(motorIn1, 0); // channel 0
  ledcAttachPin(motorIn2, 1); // channel 1
  ledcSetup(0, 1000, 8);      // 1 kHz, 8-bit
  ledcSetup(1, 1000, 8);
}

// === MOTOR CONTROL ===
void driveMotor(double pwm) {
  pwm = constrain(pwm, 0, 255);
  ledcWrite(0, pwm);
  ledcWrite(1, 0);
}

void stopMotor() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

// === SERIAL INPUT HANDLER ===
void handleSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) {
      systemRunning = false;
      stopMotor();
      Serial.println("[System Paused] Enter new values in format: Kp=0.3 Ki=0.01 Kd=0.2 set=80");
      return;
    }

    // Parse one-liner
    if (!systemRunning) {
      double newKp = Kp;
      double newKi = Ki;
      double newKd = Kd;
      double newSet = targetRPM;

      int idx;

      idx = input.indexOf("Kp=");
      if (idx != -1) newKp = input.substring(idx + 3).toFloat();

      idx = input.indexOf("Ki=");
      if (idx != -1) newKi = input.substring(idx + 3).toFloat();

      idx = input.indexOf("Kd=");
      if (idx != -1) newKd = input.substring(idx + 3).toFloat();

      idx = input.indexOf("set=");
      if (idx != -1) newSet = input.substring(idx + 4).toFloat();

      Kp = newKp;
      Ki = newKi;
      Kd = newKd;
      targetRPM = newSet;

      motorPID.SetTunings(Kp, Ki, Kd);
      systemRunning = true;

      Serial.printf("Resumed with Kp=%.3f Ki=%.3f Kd=%.3f set=%.1f\n", Kp, Ki, Kd, targetRPM);
    }
  }
}

// === SETUP ===
void setup() {
  Serial.begin(9600);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);
  setupPWM();

  motorPID.SetMode(AUTOMATIC);
  motorPID.SetSampleTime(interval);
  motorPID.SetOutputLimits(0, 255);

  Serial.println("Running... (press Enter to pause)");
  Serial.println("Format to resume: Kp=0.3 Ki=0.01 Kd=0.2 set=80");
}

// === LOOP ===
void loop() {
  handleSerial();

  if (!systemRunning) {
    return; // system is paused
  }

  unsigned long now = millis();
  if (now - lastTime >= interval) {
    // Calculate RPM
    noInterrupts();
    long ticks = encoderTicks;
    encoderTicks = 0;
    interrupts();

    double revs = ticks / (double)PULSES_PER_REV;
    currentRPM = revs * (60000.0 / interval);

    // Run PID
    motorPID.Compute();

    // Drive motor
    driveMotor(pwmOutput);

    // Debug/Plot Output
    Serial.print(targetRPM);
    Serial.print(" ");
    Serial.print(currentRPM);
    Serial.print(" ");
    Serial.println(pwmOutput);

    lastTime = now;
  }
}
Kp=0.2 Ki=0.077 Kd=0.01 set=60