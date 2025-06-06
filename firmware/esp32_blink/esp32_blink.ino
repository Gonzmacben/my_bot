#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "motor_config.h"

// === MPU6050 Gyro Variables ===
MPU6050 sensor;

float giroscAngZ = 0, giroscAngZPrev = 0;
unsigned long tiempoPrevGyro = 0;
float Kp_gyro = 23.67, Ki_gyro = 6.39, Kd_gyro = 9.02;
float integral_gyro = 0;
float error_gyro = 0, prev_error_gyro = 0;

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
// Generic ISR handler
void encoderISR(int i) {
  int b_val = digitalRead(ENC_B[i]);
  encoder_count[i] += (b_val == HIGH ? 1 : -1);
}

// Individual ISR functions call the generic handler
void encoderISR0() { encoderISR(0); }
void encoderISR1() { encoderISR(1); }
void encoderISR2() { encoderISR(2); }
void encoderISR3() { encoderISR(3); }

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

  // Initialize MPU6050
  Wire.begin();
  sensor.initialize();

  if (sensor.testConnection())
    Serial.println("MPU6050 iniciado correctamente");
  else
    Serial.println("Error al iniciar MPU6050");

  tiempoPrevGyro = millis();

  Serial.println("Arduino motor control ready");
}

void loop() {
  // Handle serial commands
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
      switchLinearActuators();
    }
  }

  // PID control loop timing
  unsigned long now = millis();
  if (now - last_pid_time >= PID_INTERVAL_MS) {
    last_pid_time = now;

    // Calculate dt for gyro
    unsigned long currentTimeGyro = millis();
    float dtGyro = (currentTimeGyro - tiempoPrevGyro) / 1000.0;
    tiempoPrevGyro = currentTimeGyro;
    if (dtGyro <= 0) dtGyro = 0.001;

    // Read gyro Z axis rotation
    int gx, gy, gz;
    sensor.getRotation(&gx, &gy, &gz);

    // Integrate gyro Z rate to get angle
    giroscAngZ = (gz / 131.0) * dtGyro + giroscAngZPrev;
    giroscAngZPrev = giroscAngZ;

    // Compute gyro PID error (target angle = 0)
    error_gyro = 0 - giroscAngZ;
    integral_gyro += error_gyro * dtGyro;
    const float integral_limit = 1000.0;
    if (integral_gyro > integral_limit) integral_gyro = integral_limit;
    if (integral_gyro < -integral_limit) integral_gyro = -integral_limit;

    float p_gyro = Kp_gyro * error_gyro;
    float i_gyro = Ki_gyro * integral_gyro;
    float d_gyro = Kd_gyro * (error_gyro - prev_error_gyro) / dtGyro;
    float PID_gyro = p_gyro + i_gyro + d_gyro;
    prev_error_gyro = error_gyro;

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

      // Add gyro PID correction to motor PWM
      float output_with_gyro;
      if (i % 2 == 0) {  // Assuming even index = left motors
        output_with_gyro = output + PID_gyro;
      } else {           // Odd index = right motors
        output_with_gyro = output - PID_gyro;
      }

      int pwm = constrain((int)output_with_gyro, -255, 255);
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
