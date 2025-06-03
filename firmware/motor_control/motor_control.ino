#include <Arduino.h>

// Motor-Encoder PINs
#define MOTOR_IN1 9
#define MOTOR_IN2 6
#define ENC_A 2
#define ENC_B 3

volatile long encoder_count = 0;
int16_t target_speed_ticks = 0;  // ticks per PID interval

// PID Parameters
float Kp = 10.0;
float Ki = 0.025;
float Kd = 1.6;
float Ko = 1.0;

float integral = 0;
float prev_error = 0;

unsigned long last_pid_time = 0;
const unsigned long PID_INTERVAL = 33; // ms (~30 Hz)

bool pid_enabled = false;

// Invert encoder direction if needed
const bool invert_encoder_direction = false;

void encoderISR() {
  int b_val = digitalRead(ENC_B);
  if (invert_encoder_direction) {
    encoder_count += (b_val == HIGH) ? -1 : 1;
  } else {
    encoder_count += (b_val == HIGH) ? 1 : -1;
  }
}

void setMotor(int pwm_val) {
  int pwm = abs(pwm_val);
  pwm = constrain(pwm, 0, 255);
  if (pwm < 40 && pwm_val != 0) pwm = 40;  // PWM threshold

  if (pwm_val > 0) {
    analogWrite(MOTOR_IN1, pwm);
    digitalWrite(MOTOR_IN2, LOW);
  } else if (pwm_val < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    analogWrite(MOTOR_IN2, pwm);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  }
}

void setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  Serial.begin(57600);
  Serial.println("Arduino motor control ready");
}

void loop() {
  // Read serial commands from ROS node
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("V:")) {
      // Velocity command
      target_speed_ticks = line.substring(2).toInt();
      pid_enabled = true;
      Serial.print("PID target speed set to: ");
      Serial.println(target_speed_ticks);
    } else if (line == "STOP") {
      pid_enabled = false;
      setMotor(0);
      Serial.println("PID disabled, motor stopped");
    }
  }

  // PID control loop
  unsigned long now = millis();
  if (pid_enabled && (now - last_pid_time >= PID_INTERVAL)) {
    last_pid_time = now;

    static long last_count = 0;
    long delta_ticks = encoder_count - last_count;
    last_count = encoder_count;

    float error = (float)target_speed_ticks - (float)delta_ticks;

    integral += error;
    float derivative = error - prev_error;
    prev_error = error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    output *= Ko;

    int pwm = constrain((int)output, -255, 255);

    setMotor(pwm);

    // Send encoder count back to ROS
    Serial.print("ENC:");
    Serial.println(encoder_count);

    // Debug info (optional)
    Serial.print("Target:");
    Serial.print(target_speed_ticks);
    Serial.print(" Actual:");
    Serial.print(delta_ticks);
    Serial.print(" Error:");
    Serial.print(error);
    Serial.print(" PWM:");
    Serial.println(pwm);
  }
}
