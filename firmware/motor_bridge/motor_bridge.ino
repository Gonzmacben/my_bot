#include "commands.h"

// Motor-Encoder PINs
#define MOTOR_IN1 9
#define MOTOR_IN2 6
#define ENC_A 2
#define ENC_B 3

// Global Variables
volatile long encoder_count = 0;
int16_t target_speed_ticks = 0;  // ticks per PID loop (speed)
int pwm_output = 0;

// PID Parameters
float Kp = 15;
float Ki = 0.035;
float Kd = 1.5;
float Ko = 1.0;

float integral = 0;
float prev_error = 0;

unsigned long last_pid_time = 0;
const unsigned long PID_INTERVAL = 33; // ms -> ~30 Hz

bool pid_enabled = false;  // Flag to activate/deactivate PID

// Serial command buffer
#define CMD_BUFFER_SIZE 64
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

// Invert encoder direction if counts are reversed after wiring swap
const bool invert_encoder_direction = false;

// Encoder ISR (increments or decrements depending on direction)
void encoderISR() {
  int b_val = digitalRead(ENC_B);
  if (invert_encoder_direction) {
    encoder_count += (b_val == HIGH) ? -1 : 1;
  } else {
    encoder_count += (b_val == HIGH) ? 1 : -1;
  }
}

// Set motor PWM and direction with PWM threshold of 40
void setMotor(int pwm_val) {
  int pwm = abs(pwm_val);
  pwm = constrain(pwm, 0, 255);
  if (pwm < 40 && pwm_val != 0) pwm = 40;  // Threshold set to 40

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

// Process a full command line
void handleCommand(String cmdLine) {
  cmdLine.trim();
  if (cmdLine.length() == 0) return;  // Ignore empty lines

  char command = cmdLine.charAt(0);

  // Validate command letter
  if (command != MOTOR_SPEEDS && command != MOTOR_PWM_RAW && command != READ_ENCODERS && command != RESET_ENCODERS) {
    Serial.print("Unknown command: ");
    Serial.println(command);
    return;
  }

  switch (command) {
    case MOTOR_SPEEDS: {  // 'm'
      int spaceIndex = cmdLine.indexOf(' ');
      if (spaceIndex > 0) {
        String arg = cmdLine.substring(spaceIndex + 1);
        arg.trim();
        if (arg.length() > 0) {
          int speed = arg.toInt();
          target_speed_ticks = speed;
          pid_enabled = true;
          Serial.print("PID enabled with target speed: ");
          Serial.println(speed);
        } else {
          Serial.println("Error: 'm' command requires speed argument");
        }
      } else {
        Serial.println("Error: 'm' command requires speed argument");
      }
      break;
    }

    case MOTOR_PWM_RAW: {  // 'o'
      int spaceIndex = cmdLine.indexOf(' ');
      if (spaceIndex > 0) {
        String arg = cmdLine.substring(spaceIndex + 1);
        arg.trim();
        if (arg.length() > 0) {
          int pwm = arg.toInt();
          pid_enabled = false;
          setMotor(pwm);
          Serial.print("Raw PWM set to: ");
          Serial.println(pwm);
        } else {
          Serial.println("Error: 'o' command requires PWM argument");
        }
      } else {
        Serial.println("Error: 'o' command requires PWM argument");
      }
      break;
    }

    case READ_ENCODERS: {  // 'e'
      Serial.print("Encoder count: ");
      Serial.println(encoder_count);
      break;
    }

    case RESET_ENCODERS: {  // 'r'
      encoder_count = 0;
      integral = 0;
      prev_error = 0;
      Serial.println("Encoder and PID reset");
      break;
    }
  }
}

// Read and process serial commands (buffer until newline)
void processCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    // Echo typed characters
    Serial.write(c);

    if (c == '\r') continue;  // Ignore carriage return

    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';  // Null-terminate string
      String cmdLine = String(cmdBuffer);
      handleCommand(cmdLine);
      cmdIndex = 0;  // Reset buffer
    } else {
      if (cmdIndex < CMD_BUFFER_SIZE - 1) {
        cmdBuffer[cmdIndex++] = c;
      } else {
        cmdIndex = 0;
        Serial.println("Error: command too long");
      }
    }
  }
}

// PID loop to adjust PWM based on target speed, with debug output including PWM
void runPID() {
  unsigned long now = millis();
  if (now - last_pid_time >= PID_INTERVAL) {
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

    int pwm = (int)output;
    pwm = constrain(pwm, -255, 255);

    setMotor(pwm);

    // Debug print: target speed, actual speed, error, and pwm
    Serial.print("Target: ");
    Serial.print(target_speed_ticks);
    Serial.print("  Actual: ");
    Serial.print(delta_ticks);
    Serial.print("  Error: ");
    Serial.print(error);
    Serial.print("  PWM: ");
    Serial.println(pwm);
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
  processCommand();

  if (pid_enabled) {
    runPID();
  }
}
