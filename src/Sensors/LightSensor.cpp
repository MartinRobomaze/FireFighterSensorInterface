#include "LightSensor.h"


LightSensor::LightSensor(int lightSensorChan) {
    LightSensor::lightSensorChan = lightSensorChan;
}

int LightSensor::read() {
    byte config =
            (1 << 7) |  // Single ended inputs
            (LightSensor::chanToADS7830Chan[LightSensor::lightSensorChan] << 4) |   // Channel selection
            (1 << 3) |  // PD Mode selection - Ref & AD converter on
            (1 << 2);

    // Write config byte.
    Wire.beginTransmission(ADS7830ADDR);
    Wire.write((uint8_t)config);
    Wire.endTransmission();

    // Wait for conversion to finish.
    delayMicroseconds(CONVERSION_DELAY_US);

    // Read sensor data.
    Wire.requestFrom(ADS7830ADDR, 1);
    return Wire.read();
}