#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// Number of PID-controlled motors
const int NUM_MOTORS = 4;

// Motor pins with position labels
const int MOTOR_IN1[NUM_MOTORS] = {
  4,  // FL - Front Left motor PWM pin
  5,  // FR - Front Right motor PWM pin
  6,  // BL - Back Left motor PWM pin
  7   // BR - Back Right motor PWM pin
};

const int MOTOR_IN2[NUM_MOTORS] = {
  8,  // FL - Front Left motor direction pin
  9,  // FR - Front Right motor direction pin
  10, // BL - Back Left motor direction pin
  11  // BR - Back Right motor direction pin
};

// Encoder pins for each motor
const int ENC_A[NUM_MOTORS] = {
  3,  // FL encoder channel A
  2,  // FR encoder channel A
  18, // BL encoder channel A
  19  // BR encoder channel A
};

const int ENC_B[NUM_MOTORS] = {
  14, // FL encoder channel B
  15, // FR encoder channel B
  16, // BL encoder channel B
  17  // BR encoder channel B
};

// Linear actuator pins (adjust as needed) 
const int LINEAR_ACTUATOR_IN1[2] = {22, 24};   // Forward control pins 
const int LINEAR_ACTUATOR_IN2[2] = {26, 28}; // Reverse control pins

// IMU PINS MUST BE 0,1

// PID constants per motor (example values)
const float Kp[NUM_MOTORS] = {2.5, 2.5, 2.5, 2.5};
const float Ki[NUM_MOTORS] = {0.8, 0, 0.8, 0}; //
const float Kd[NUM_MOTORS] = {0.6, 0.6, 0.6, 0.6};
const float Ko[NUM_MOTORS] = {1.0, 1.0, 1.0, 1.0};

// Encoder inversion flags
const bool invert_encoder[NUM_MOTORS] = {false, false, false, false};

// Encoder and timing constants
const float TICKS_PER_REV = 1414.7;
const float MS_PER_MIN = 60000.0;
const unsigned long PID_INTERVAL = 33;  // ~30Hz PID loop

#endif // MOTOR_CONFIG_H
