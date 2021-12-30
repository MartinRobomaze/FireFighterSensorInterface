#include "LightSensor.h"


LightSensor::LightSensor(int lightSensorPin, int resolution) {
  LightSensor::lightSensorPin = lightSensorPin;
  LightSensor::resolution = resolution;

  // Set resolution of the analog reading.
  analogReadResolution(resolution);
}

int LightSensor::read() {
  // Read analog value of the sensor.
  int reading = analogRead(LightSensor::lightSensorPin);

  return reading;
}