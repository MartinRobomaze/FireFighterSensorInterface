#include "Motor.h"


Motor::Motor(int *ledcChannels) {
  Motor::ledcChannels = ledcChannels;
}

void Motor::motorWrite(char direction, int speed) {
  // Turn motor forward.
  if (direction == 'F') {
    ledcWrite(Motor::ledcChannels[0], speed);
    ledcWrite(Motor::ledcChannels[1], 0);
  }

  // Turn motor backward.
  else if (direction == 'B') {
    ledcWrite(Motor::ledcChannels[0], 0);
    ledcWrite(Motor::ledcChannels[1], speed);
  }
}

void Motor::brake() {
  ledcWrite(Motor::ledcChannels[0], 255);
  ledcWrite(Motor::ledcChannels[1], 255);
}