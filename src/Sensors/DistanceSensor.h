#ifndef FIREFIGHTERSENSORINTERFACE_DISTANCESENSOR_H
#define FIREFIGHTERSENSORINTERFACE_DISTANCESENSOR_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

enum DistanceSensorError {
    NoError,
    AddrSetError,
    MeasurementSetError
};

class DistanceSensor {
public:
    /**
     * VL53L0X Single sensor reader.
     * @param i2cAddress I2C address of the sensor.
     * @param measurementPeriod Period between sensor measurements.
     */
    DistanceSensor(uint8_t i2cAddress, int measurementPeriod);

    /**
     * Initializes communication with the sensor.
     * @return true if init was successful.
     */
    DistanceSensorError begin();

    /**
     * Performs a range measurement and returns the result.
     * @return Distance of range measurement in mm.
     */
    uint16_t readDistance();

private:
    uint8_t i2cAddress;
    int measurementPeriod;
    Adafruit_VL53L0X sensor;
};

#endif
