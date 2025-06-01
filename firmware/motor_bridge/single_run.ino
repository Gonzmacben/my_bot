#define MOTOR_IN1 9
#define MOTOR_IN2 6

void setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
}

void loop() {
  analogWrite(MOTOR_IN1, 255);
  digitalWrite(MOTOR_IN2, LOW);
  delay(2000);

  analogWrite(MOTOR_IN1, 0);
  digitalWrite(MOTOR_IN2, LOW);
  delay(2000);
}