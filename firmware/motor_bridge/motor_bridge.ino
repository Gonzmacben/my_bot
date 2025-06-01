#include "commands.h"

// Pines motor y encoder
#define MOTOR_IN1 9
#define MOTOR_IN2 6
#define ENC_A 2
#define ENC_B 3

// Variables globales
volatile long encoder_count = 0;
int16_t target_speed_ticks = 0;  // objetivo de ticks por loop PID (velocidad)
int pwm_output = 0;

// Parámetros PID
float Kp = 20.0;
float Ki = 0.0;
float Kd = 12.0;
float Ko = 50.0;

float integral = 0;
float prev_error = 0;

unsigned long last_pid_time = 0;
const unsigned long PID_INTERVAL = 33; // ms -> ~30 Hz

bool pid_enabled = false;  // Flag para activar/desactivar PID

// ISR encoder (incrementa o decrementa según dirección)
void encoderISR() {
  int b_val = digitalRead(ENC_B);
  encoder_count += (b_val == HIGH) ? 1 : -1;
}

// Función para establecer PWM y dirección del motor
void setMotor(int pwm_val) {
  int pwm = abs(pwm_val);
  pwm = constrain(pwm, 0, 255);
  if (pwm < 80 && pwm_val != 0) pwm = 80;

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

// Leer y procesar comandos seriales
void processCommand() {
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case MOTOR_SPEEDS: {  // 'm'
        // Comando 'm' para velocidad PID (2 bytes)
        while (Serial.available() < 2);
        int16_t speed = Serial.read() | (Serial.read() << 8);
        target_speed_ticks = speed;
        pid_enabled = true;  // activar PID
        break;
      }
      case 'o': {
        // Comando 'o' para PWM raw (1 byte)
        while (Serial.available() < 1);
        int8_t pwm = Serial.read();  // signed 8 bits, -128 a 127
        pid_enabled = false;  // desactivar PID
        setMotor(pwm);
        break;
      }
      case READ_ENCODERS: {
        // Enviar encoder (4 bytes, little endian)
        long count = encoder_count;
        Serial.write((uint8_t*)&count, sizeof(count));
        break;
      }
      case RESET_ENCODERS: {
        encoder_count = 0;
        integral = 0;
        prev_error = 0;
        break;
      }
    }
  }
}

// Ejecutar PID para ajustar PWM según objetivo de velocidad
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
  }
}

void setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  Serial.begin(57600);
}

void loop() {
  processCommand();

  if (pid_enabled) {
    runPID();
  }
}