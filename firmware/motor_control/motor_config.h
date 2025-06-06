#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// Number of PID-controlled motors
const int NUM_MOTORS = 4;

// Number of Linear Actuators
const int NUM_ACTUATORS = 2;

// Motor control pins (PWM and direction)
const int MOTOR_IN1[NUM_MOTORS] = {4, 5, 6, 7};  // Motor 1 to 4 PWM pins
const int MOTOR_IN2[NUM_MOTORS] = {8, 9, 10, 11}; // Motor 1 to 4 direction pins

// Encoder pins for each motor
const int ENC_A[NUM_MOTORS] = {3, 2, 18, 19};  // Encoder channel A pins
const int ENC_B[NUM_MOTORS] = {14, 15, 16, 17}; // Encoder channel B pins

// Linear actuator control pins
const int LINEAR_ACTUATOR_IN1[NUM_ACTUATORS] = {22, 24};  // Linear actuator forward control pins
const int LINEAR_ACTUATOR_IN2[NUM_ACTUATORS] = {18, 19}; // Linear actuator reverse control pins

// Encoder and timing constants
const long TICKS_PER_REV = 3018;       // Encoder ticks per revolution
const unsigned long PID_INTERVAL_MS = 33;  // PID loop interval in milliseconds (~30Hz)

// PID constants per motor (optional, if you add PID control later)
const float Kp[NUM_MOTORS] = {2.5, 2.5, 2.5, 2.5};
const float Ki[NUM_MOTORS] = {0.8, 0.8, 0.8, 0.8};
const float Kd[NUM_MOTORS] = {0.6, 0.6, 0.6, 0.6};
const float Ko[NUM_MOTORS] = {1.0, 1.0, 1.0, 1.0};

#endif // MOTOR_CONFIG_H
