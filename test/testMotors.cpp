#include <Arduino.h>
#include <Motors/Motor.h>


int motorPins[8] = {19, 18, 5, 17, 16, 4, 2, 0};
int motorChannels[4][2] = {
  {0, 1},
  {2, 3},
  {4, 5},
  {6, 7}
};

// Creates motor class instance.
Motor motors[4] = {
  Motor(motorChannels[0]),
  Motor(motorChannels[1]),
  Motor(motorChannels[2]),
  Motor(motorChannels[3])
};

void setup() {
  // Motor pins setup.

  int k = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
      pinMode(motorPins[k], OUTPUT);
      ledcSetup(motorChannels[i][j], 1000, 8);
      ledcAttachPin(motorPins[k], motorChannels[i][j]);
      k++;
    }
  }
}

void loop() {
  digitalWrite(2, LOW);
  digitalWrite(0, LOW);
  motors[0].motorWrite('F', 255);
  motors[1].motorWrite('F', 255);
}