#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(uint8_t address, int measurementPeriod) {
    DistanceSensor::i2cAddress = address;
    DistanceSensor::measurementPeriod = measurementPeriod;
}

DistanceSensorError DistanceSensor::begin() {
    if (!DistanceSensor::sensor.begin(DistanceSensor::i2cAddress)) {
        return AddrSetError;
    }

    if (!DistanceSensor::sensor.startRangeContinuous()) {
        return MeasurementSetError;
    }

    return NoError;
}

uint16_t DistanceSensor::readDistance() {
    return DistanceSensor::sensor.readRangeResult();
}
