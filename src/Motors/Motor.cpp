#include "Motor.h"


Motor::Motor(MotorPins pins) {
  ledcSetup(pins.channel1, 1000, 8);
  ledcAttachPin(pins.pin1, pins.channel1);
  ledcSetup(pins.channel2, 1000, 8);
  ledcAttachPin(pins.pin2, pins.channel2);

  Motor::pins = pins;
}

void Motor::motorWrite(int speed) {
  // Turn motor forward.
  if (speed > 0) {
    ledcWrite(Motor::pins.channel1, speed);
    ledcWrite(Motor::pins.channel2, 0);
  } else {
    ledcWrite(Motor::pins.channel1, 0);
    ledcWrite(Motor::pins.channel2, abs(speed));
  }
}

void Motor::brake() {
  ledcWrite(Motor::pins.channel1, 255);
  ledcWrite(Motor::pins.channel2, 255);
}