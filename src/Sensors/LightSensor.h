/**
 * Low level analog light sensor reader. By default it uses 12 bit resolution, it can be changed. 
 * 
 * Input: Pin of the light sensor to be read.
 * Output: The sensor's reading.
 * 
 * Creator: Martinlinux
 * Version: 1.0
 */

#include <Arduino.h>


class LightSensor {
  public:
    ///<summary> Low level analog light sensor reader.
    ///<param name="lightSensorPin"> Pin of the light sensor.
    ///<param name="resolution"> Optional, the resolution of reading, by default it's 12 bit.
    LightSensor(int lightSensorPin, int resolution = 12);

    ///<summary> Reads analog value of the sensor.
    int read();
  private:
    int lightSensorPin;
    int resolution;
};
