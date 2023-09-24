#ifndef FIREFIGHTERSENSORINTERFACE_MOTOR_H
#define FIREFIGHTERSENSORINTERFACE_MOTOR_H

#include <Arduino.h>

struct MotorPins {
  u_int8_t pin1;
  u_int8_t pin2;
  u_int8_t channel1;
  u_int8_t channel2;
};

class Motor {
  public:

    // @brief Sets up motor 
    Motor(MotorPins &pins);

    /// <summary> Turns motor to a given direction, at a given speed.
    /// <param name="direction"> Direction for the motor to be turned.
    /// <param name="speed"> Speed for the motor to be turned.
    void motorWrite(int speed);

    void brake();
  private:
    MotorPins pins;
};

#endif
