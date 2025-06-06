#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// Number of PID-controlled motors
const int NUM_MOTORS = 4;

// Number of Linear Actuators
const int NUM_ACTUATORS = 2;

// Pins for 4 PID motors
const int MOTOR_IN1[NUM_MOTORS] = {4, 5, 6, 7}; // FL, FR, BL, BR
const int MOTOR_IN2[NUM_MOTORS] = {8, 9, 10, 11}; // ---
const int ENC_A[NUM_MOTORS] = {3, 2, 18, 19}; // ---
const int ENC_B[NUM_MOTORS] = {14, 15, 16, 17}; // ---

// IMU I2C pins (if needed for manual Wire setup)
const int IMU_SDA_PIN = 20;
const int IMU_SCL_PIN = 21;

// PID parameters per motor (adjust as needed)
const float Kp[NUM_MOTORS] = {2.5, 2.5, 2.5, 2.5};
const float Ki[NUM_MOTORS] = {0.8, 0.8, 0.8, 0.8};
const float Kd[NUM_MOTORS] = {0.6, 0.6, 0.6, 0.6};
const float Ko[NUM_MOTORS] = {1.0, 1.0, 1.0, 1.0};

// Encoder and timing constants
const float TICKS_PER_REV = 3018.0;
const float MS_PER_MIN = 60000.0;
const unsigned long PID_INTERVAL_MS = 33; // PID loop interval (~30Hz)

// Pins for 2 linear DC motors controlled by L298N
const int LINEAR_ACTUATOR_IN1[NUM_ACTUATORS] = {0, 1};  // Forward control pins
const int LINEAR_ACTUATOR_IN2[NUM_ACTUATORS] = {18, 19}; // Reverse control pins

#endif // MOTOR_CONFIG_H
