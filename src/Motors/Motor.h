/**
 * Low level motor controller using PWM via ledc on ESP32.
 * Input: ledc channels, that are already setup.
 * Output: Turning motors on and off at a variable speed.
 * 
 * Creator: Martinlinux
 * Version: 1.0
 */

#include <Arduino.h>


class Motor {
  public:

    /// <summary> Low level motor controller
    /// <param name="ledcChannels"> Ledc channels, that are already setup.
    Motor(int* ledcChannels);

    /// <summary> Turns motor to a given direction, at a given speed.
    /// <param name="direction"> Direction for the motor to be turned.
    /// <param name="speed"> Speed for the motor to be turned.
    void motorWrite(char direction, int speed);

    void brake();
  private:
    int *ledcChannels;
};