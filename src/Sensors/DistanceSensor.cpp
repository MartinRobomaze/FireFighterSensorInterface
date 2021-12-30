#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(uint8_t mcp23071Addr, uint8_t echoPin, uint8_t trigPin) {
    DistanceSensor::echoPin = echoPin;
    DistanceSensor::trigPin = trigPin;

    DistanceSensor::expaderHandle = MCP23017(mcp23071Addr);

    DistanceSensor::expaderHandle.pinMode(echoPin, INPUT_VAL);
    DistanceSensor::expaderHandle.pinMode(trigPin, OUTPUT_VAL);
}

uint8_t DistanceSensor::readDistance() {
    Serial.println("mopslik1");
    DistanceSensor::expaderHandle.write(trigPin, 0);
    delayMicroseconds(2);

    DistanceSensor::expaderHandle.write(trigPin, 1);
    delayMicroseconds(10);
    Serial.println("mopslik2");

    DistanceSensor::expaderHandle.write(trigPin, 0);
    Serial.println("mopslik3");

    unsigned long timeStart = micros();
    unsigned long duration = 0;

    uint8_t echoPinVal = 0;
    while (true) {
        DistanceSensor::expaderHandle.read(echoPin, &echoPinVal);
        if (echoPinVal == 1 || micros() - timeStart > 1000000) {
            duration = micros() - timeStart;

            break;
        }
    }

    return (uint8_t)((double)(duration) * 0.034 / 2);
}
