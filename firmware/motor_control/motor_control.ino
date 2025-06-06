#include <Arduino.h>
#include "motor_config.h"

// === PID Motors Variables ===
volatile long encoder_count[NUM_MOTORS] = {0};
float target_speed_rpm[NUM_MOTORS] = {0};
float integral[NUM_MOTORS] = {0};
float prev_error[NUM_MOTORS] = {0};
bool pid_enabled[NUM_MOTORS] = {false};

unsigned long last_pid_time = 0;

// === Linear Motors Control ===

void setLinearActuator(int i, int pwm_val) {
  int pwm = abs(pwm_val);
  pwm = constrain(pwm, 0, 255);

  if (pwm_val > 0) {
    analogWrite(LINEAR_MOTOR_IN1[i], pwm);
    digitalWrite(LINEAR_MOTOR_IN2[i], LOW);
  } else if (pwm_val < 0) {
    digitalWrite(LINEAR_MOTOR_IN1[i], LOW);
    analogWrite(LINEAR_MOTOR_IN2[i], pwm);
  } else {
    digitalWrite(LINEAR_MOTOR_IN1[i], LOW);
    digitalWrite(LINEAR_MOTOR_IN2[i], LOW);
  }
}

void extendLinearActuators() {
  setLinearActuator(0, 255);
  setLinearActuator(1, 255);
}

void retractLinearActuators() {
  setLinearActuator(0, -255);
  setLinearActuator(1, -255);
}

void stopLinearActuators() {
  setLinearActuator(0, 0);
  setLinearActuator(1, 0);
}

void switchLinearActuators() {
  extendLinearActuators();
  delay(10000); // Extend for 10 seconds
  stopLinearActuators();
  delay(500);
  retractLinearActuators();
  delay(10000); // Retract for 10 seconds
  stopLinearActuators();
}

// === PID Motors Functions ===

// Encoder ISRs for each motor
void encoderISR0() {
  int b_val = digitalRead(ENC_B[0]);
  encoder_count[0] += (b_val == HIGH ? 1 : -1);
}

void encoderISR1() {
  int b_val = digitalRead(ENC_B[1]);
  encoder_count[1] += (b_val == HIGH ? 1 : -1);
}

void encoderISR2() {
  int b_val = digitalRead(ENC_B[2]);
  encoder_count[2] += (b_val == HIGH ? 1 : -1);
}

void encoderISR3() {
  int b_val = digitalRead(ENC_B[3]);
  encoder_count[3] += (b_val == HIGH ? 1 : -1);
}

void setMotor(int i, int pwm_val) {
  int pwm = abs(pwm_val);
  pwm = constrain(pwm, 0, 255);
  if (pwm < 40 && pwm_val != 0) pwm = 40;

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

void setup() {
  Serial.begin(57600);

  // Setup PID motors pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }

  // Setup linear motors pins
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    pinMode(LINEAR_ACTUATOR_IN1[i], OUTPUT);
    pinMode(LINEAR_ACTUATOR_IN2[i], OUTPUT);
  }

  // Attach interrupts for encoder channels
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), encoderISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), encoderISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), encoderISR3, RISING);

  Serial.println("Arduino motor control ready");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("RESET")) {
      for (int i = 0; i < NUM_MOTORS; i++) encoder_count[i] = 0;
      Serial.println("RESET_OK");

    } else if (input.startsWith("RPM:")) {
      String data = input.substring(4);
      for (int i = 0; i < NUM_MOTORS; i++) {
        int sep = data.indexOf(',');
        String val = (sep == -1) ? data : data.substring(0, sep);
        target_speed_rpm[i] = val.toFloat();
        pid_enabled[i] = true;
        if (sep == -1) break;
        data = data.substring(sep + 1);
      }

    } else if (input.startsWith("PWM:")) {
      String data = input.substring(4);
      for (int i = 0; i < NUM_MOTORS; i++) {
        int sep = data.indexOf(',');
        String val = (sep == -1) ? data : data.substring(0, sep);
        int pwm_val = val.toInt();
        pid_enabled[i] = false;
        setMotor(i, pwm_val);
        if (sep == -1) break;
        data = data.substring(sep + 1);
      }

    } else if (input.equalsIgnoreCase("STOP")) {
      for (int i = 0; i < NUM_MOTORS; i++) {
        pid_enabled[i] = false;
        setMotor(i, 0);
        integral[i] = 0;
        prev_error[i] = 0;
      }
      stopLinearActuators();

    } else if (input.equalsIgnoreCase("SWITCH")) {
      // Run the linear actuator extend/retract sequence
      switchLinearActuators();
    }
  }

  // PID control loop
  unsigned long now = millis();
  if (now - last_pid_time >= PID_INTERVAL_MS) {
    last_pid_time = now;

    static long last_counts[NUM_MOTORS] = {0};
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (!pid_enabled[i]) continue;

      long delta = encoder_count[i] - last_counts[i];
      last_counts[i] = encoder_count[i];

      float measured_rpm = ((float)delta / TICKS_PER_REV) * (MS_PER_MIN / PID_INTERVAL_MS);
      float error = target_speed_rpm[i] - measured_rpm;

      integral[i] += error;
      float derivative = error - prev_error[i];
      prev_error[i] = error;

      float output = Kp[i] * error + Ki[i] * integral[i] + Kd[i] * derivative;
      output *= Ko[i];

      int pwm = constrain((int)output, -255, 255);
      setMotor(i, pwm);
    }

    // Send encoder counts via serial
    Serial.print("ENC:");
    for (int i = 0; i < NUM_MOTORS; i++) {
      Serial.print(encoder_count[i]);
      if (i < (NUM_MOTORS - 1)) Serial.print(",");
    }
    Serial.println();
  }
}
