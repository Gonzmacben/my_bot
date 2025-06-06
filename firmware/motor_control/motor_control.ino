#include <Arduino.h>
#include "motor_config.h"

// Encoder counts and PID variables
volatile long encoder_count[NUM_MOTORS] = {0};
float target_speed_rpm[NUM_MOTORS] = {0};
float integral[NUM_MOTORS] = {0};
float prev_error[NUM_MOTORS] = {0};
bool pid_enabled[NUM_MOTORS] = {false};

unsigned long last_pid_time = 0;

// Direction inversion flags: invert FR (index 1) and BR (index 3) motors
const bool invert_motor_dir[NUM_MOTORS] = {false, true, false, true};

// Encoder ISR handler
void encoderISR(int i) {
  int b_val = digitalRead(ENC_B[i]);
  encoder_count[i] += (invert_encoder[i] ? (b_val == HIGH ? -1 : 1) : (b_val == HIGH ? 1 : -1));
}

void encoderISR0() { encoderISR(0); }
void encoderISR1() { encoderISR(1); }
void encoderISR2() { encoderISR(2); }
void encoderISR3() { encoderISR(3); }

// Motor control for 4 motors with direction inversion
void setMotor(int i, int pwm_val) {
  if (invert_motor_dir[i]) pwm_val = -pwm_val;

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

// Linear actuator control via L298N driver (2 actuators)
void setLinearActuator(int actuator_num, int pwm_val) {
  if (actuator_num < 0 || actuator_num > 1) return;

  int in1_pin = LINEAR_ACTUATOR_IN1[actuator_num];
  int in2_pin = LINEAR_ACTUATOR_IN2[actuator_num];

  int pwm = abs(pwm_val);
  pwm = constrain(pwm, 0, 255);
  if (pwm < 40 && pwm_val != 0) pwm = 40;

  if (pwm_val > 0) {
    analogWrite(in1_pin, pwm);
    digitalWrite(in2_pin, LOW);
  } else if (pwm_val < 0) {
    digitalWrite(in1_pin, LOW);
    analogWrite(in2_pin, pwm);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
  }
}

void stopLinearActuators() {
  for (int i = 0; i < 2; i++) {
    setLinearActuator(i, 0);
  }
}

void setup() {
  Serial.begin(57600);

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }

  for (int i = 0; i < 2; i++) {
    pinMode(LINEAR_ACTUATOR_IN1[i], OUTPUT);
    pinMode(LINEAR_ACTUATOR_IN2[i], OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), encoderISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), encoderISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), encoderISR3, RISING);

  Serial.println("4-motor PID control with linear actuators ready");
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

    } else if (input.startsWith("ACT:")) {
      String data = input.substring(4);
      for (int i = 0; i < 2; i++) {
        int sep = data.indexOf(',');
        String val = (sep == -1) ? data : data.substring(0, sep);
        int pwm_val = val.toInt();
        setLinearActuator(i, pwm_val);
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
      setLinearActuator(0, 255);
      setLinearActuator(1, 255);

    } else if (input.equalsIgnoreCase("SWITCH_BACK")) {
      setLinearActuator(0, -255);
      setLinearActuator(1, -255);
    }
  }

  unsigned long now = millis();
  if (now - last_pid_time >= PID_INTERVAL) {
    last_pid_time = now;

    static long last_counts[NUM_MOTORS] = {0};
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (!pid_enabled[i]) continue;

      long delta = encoder_count[i] - last_counts[i];
      last_counts[i] = encoder_count[i];

      float measured_rpm = ((float)delta / TICKS_PER_REV) * (MS_PER_MIN / PID_INTERVAL);
      float error = target_speed_rpm[i] - measured_rpm;

      integral[i] += error;
      float derivative = error - prev_error[i];
      prev_error[i] = error;

      float output = Kp[i] * error + Ki[i] * integral[i] + Kd[i] * derivative;
      output *= Ko[i];

      int pwm = constrain((int)output, -255, 255);

      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" PWM: ");
      Serial.println(pwm);

      setMotor(i, pwm);
    }

    Serial.print("ENC:");
    for (int i = 0; i < NUM_MOTORS; i++) {
      Serial.print(encoder_count[i]);
      if (i < NUM_MOTORS - 1) Serial.print(",");
    }
    Serial.println();
  }
}
