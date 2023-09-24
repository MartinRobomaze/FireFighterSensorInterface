#ifndef FIREFIGHTERSENSORINTERFACE_LIGHTSENSOR_H
#define FIREFIGHTERSENSORINTERFACE_LIGHTSENSOR_H

#include <Arduino.h>
#include <Wire.h>

#define ADS7830ADDR 0x48
#define CONVERSION_DELAY_US 10 // Delay between requesting and receiving ADS7830 reading

class LightSensor {
public:
    /**
     * @brief Low level analog light sensor reader.
     * @param lightSensorChan Light sensor channel.
     */
    explicit LightSensor(int lightSensorChan);

    /**
     * @brief Reads the value of the light sensor.
     * @return Sensor value.
     */
    int read();

private:
    int lightSensorChan;

    // https://www.ti.com/lit/ds/symlink/ads7830.pdf?ts=1695499691046&ref_url=https%253A%252F%252Fwww.google.com%252F
    // Table no. 2
    const int chanToADS7830Chan[8] = {0, 2, 4, 6, 1, 3, 5, 7};
};

#endif