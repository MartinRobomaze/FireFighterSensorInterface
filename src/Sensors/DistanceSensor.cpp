#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(uint8_t pingPin) {
    DistanceSensor::pingPin = pingPin;
}

uint8_t DistanceSensor::readDistance() const {
    pinMode(DistanceSensor::pingPin, OUTPUT);

    digitalWrite(DistanceSensor::pingPin, LOW);
    delayMicroseconds(2);

    digitalWrite(DistanceSensor::pingPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(DistanceSensor::pingPin, LOW);

    pinMode(DistanceSensor::pingPin, INPUT);

    u_long duration = pulseIn(DistanceSensor::pingPin, HIGH);

    return (uint8_t)((double)(duration) / 29 / 2);
}
