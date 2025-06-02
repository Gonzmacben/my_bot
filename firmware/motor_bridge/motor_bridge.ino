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
          int pwm = arg.toInt();  // toInt() handles negative numbers fine
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
