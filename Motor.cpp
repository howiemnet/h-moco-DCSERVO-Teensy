#include "Motor.h"
#include "Arduino.h"

Motor::Motor() {

  digitalWrite(MOTOR_DRIVER_ENABLE, LOW);
  digitalWrite(MOTOR_DRIVER_GROUND, LOW);
  digitalWrite(MOTOR_DRIVER_LEFT_PWM, LOW);
  digitalWrite(MOTOR_DRIVER_RIGHT_PWM, LOW);

  pinMode(MOTOR_DRIVER_ENABLE, OUTPUT);
  pinMode(MOTOR_DRIVER_GROUND, OUTPUT);
  pinMode(MOTOR_DRIVER_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_DRIVER_RIGHT_PWM, OUTPUT);

  analogWriteFrequency(MOTOR_DRIVER_LEFT_PWM, MOTOR_PWM_FREQUENCY);
  analogWriteFrequency(MOTOR_DRIVER_RIGHT_PWM, MOTOR_PWM_FREQUENCY);
  analogWriteResolution(MOTOR_PWM_RESOLUTION);
}

void Motor::enableMotor() {
  setPWM(0.0);
  digitalWrite(MOTOR_DRIVER_ENABLE, HIGH);
  motorEnabled = true;
}

void Motor::disableMotor() {
  digitalWrite(MOTOR_DRIVER_ENABLE, LOW);
  setPWM(0.0);
  motorEnabled = false;
}




void Motor::setPWM(float PWMlevel) {
  if (motorEnabled) {
         int16_t actualPWMlevel = PWMlevel * 255;
 
    if (actualPWMlevel == 0) {      // zero power
      analogWrite(MOTOR_DRIVER_LEFT_PWM, 0);
      analogWrite(MOTOR_DRIVER_RIGHT_PWM, 0);
    }

    else {

     
      
      

      if (actualPWMlevel > 0) {    // going right
        if (actualPWMlevel > 255) { 
          actualPWMlevel = 255; 
        }
        analogWrite(MOTOR_DRIVER_LEFT_PWM, 0);
        analogWrite(MOTOR_DRIVER_RIGHT_PWM, actualPWMlevel);

      } else {               // going left
        actualPWMlevel = -actualPWMlevel;
if (actualPWMlevel > 255) { actualPWMlevel = 255; }
        
        analogWrite(MOTOR_DRIVER_RIGHT_PWM, 0);
        analogWrite(MOTOR_DRIVER_LEFT_PWM , actualPWMlevel);
        
        
      }
    }
  }
}

