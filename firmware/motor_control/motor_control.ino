#include <Arduino.h>
#include "motor_config.h"

// Encoder counts for 4 motors
volatile long encoder_count[NUM_MOTORS] = {0};

// Target encoder counts for movement phases
const long target_counts = 10000;  // Adjust as needed
const unsigned long pause_duration_ms = 3000;  // 3 seconds pause

// Motor control functions
void setMotor(int i, int pwm_val) {
  int pwm = abs(pwm_val);
  pwm = constrain(pwm, 0, 255);

  if (pwm_val > 0) {
    analogWrite(MOTOR_IN1[i], pwm);
    digitalWrite(MOTOR_IN2[i], LOW);
  } else if (pwm_val < 0) {
    digitalWrite(MOTOR_IN1[i], LOW);
    analogWrite(MOTOR_IN2[i], pwm);
  } else {
    digitalWrite(MOTOR_IN1[i], LOW);
    digitalWrite(MOTOR_IN2[i], LOW);
  }
}

void stopAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    setMotor(i, 0);
  }
}

// Encoder ISR handlers (example for 4 motors)
void encoderISR0() { encoder_count[0]++; }
void encoderISR1() { encoder_count[1]++; }
void encoderISR2() { encoder_count[2]++; }
void encoderISR3() { encoder_count[3]++; }

// Linear actuator control (example)
void extendLinearActuators();
void retractLinearActuators();
void stopLinearActuators();
void switchLinearActuators() {
  extendLinearActuators();
  delay(10000);
  stopLinearActuators();
  delay(500);
  retractLinearActuators();
  delay(10000);
  stopLinearActuators();
}

void extendLinearActuators() {
  // Implement your linear actuator extension logic here
}

void retractLinearActuators() {
  // Implement your linear actuator retraction logic here
}

void stopLinearActuators() {
  // Implement your linear actuator stop logic here
}

// Wait until all motors have moved target_counts since start_counts
bool reachedTargetCounts(long start_counts[]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (abs(encoder_count[i] - start_counts[i]) < target_counts) {
      return false;
    }
  }
  return true;
}

void resetEncoders() {
  noInterrupts();
  for (int i = 0; i < NUM_MOTORS; i++) {
    encoder_count[i] = 0;
  }
  interrupts();
}

void setup() {
  Serial.begin(57600);

  // Setup motor pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), encoderISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), encoderISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), encoderISR3, RISING);

  Serial.println("Motor control ready");
}

void loop() {
  // Phase 1: Move forward
  Serial.println("Phase 1: Moving forward");
  long start_counts[NUM_MOTORS];
  noInterrupts();
  for (int i = 0; i < NUM_MOTORS; i++) start_counts[i] = encoder_count[i];
  interrupts();

  // Set all motors forward at fixed PWM (e.g., 150)
  for (int i = 0; i < NUM_MOTORS; i++) setMotor(i, 150);

  while (!reachedTargetCounts(start_counts)) {
    // Optionally, add small delay or monitor encoders
    delay(10);
  }
  stopAllMotors();

  // Phase 2: Pause
  Serial.println("Phase 2: Pause");
  unsigned long pause_start = millis();
  while (millis() - pause_start < pause_duration_ms) {
    delay(10);
  }

  // Phase 3: Switch linear actuators
  Serial.println("Phase 3: Switching linear actuators");
  switchLinearActuators();

  // Phase 4: Move backward
  Serial.println("Phase 4: Moving backward");
  noInterrupts();
  for (int i = 0; i < NUM_MOTORS; i++) start_counts[i] = encoder_count[i];
  interrupts();

  // Set all motors backward at fixed PWM (e.g., -150)
  for (int i = 0; i < NUM_MOTORS; i++) setMotor(i, -150);

  while (!reachedTargetCounts(start_counts)) {
    delay(10);
  }
  stopAllMotors();

  // Phase 5: Final pause
  Serial.println("Phase 5: Final pause");
  pause_start = millis();
  while (millis() - pause_start < pause_duration_ms) {
    delay(10);
  }

  Serial.println("Sequence complete");
  while (true) {
    delay(1000);  // Stop here
  }
}
