#include <Arduino.h>
#include <Wire.h>
#include "Motors/Motor.h"
#include "Sensors/DistanceSensor.h"
#include "Sensors/LightSensor.h"
#include "Sensors/Encoder.h"
#include "communicationHandler.h"
#include "Sensors/IMUSensor.h"

#define DIST_SENSORS_BASE_ADDRESS 0x30
#define DIST_SENSORS_MEAS_PERIOD 10

const int enablePin = 32;

const uint8_t distSensorsSHTPins[8] = {33, 25, 26, 27, 16, 17, 18, 19};

MotorPins motorPins[4] = {
        {5, 4, 0, 1},
        {2, 15, 4, 5},
        {14, 12, 2, 3},
        {13, 23, 6, 7}
};

int imuInterruptPin = 15;
bool resetEncoders = false;

Motor *motors[4];
LightSensor *lineSensors[8];
DistanceSensor *distanceSensors[8];
Encoder *encoders[4];

//IMUSensor *mpu;

// Creates communicationHandler class instance.
CommunicationHandler commHandler;

EncodedMessage handleDataMessage() {
    SensorsData data{};
    for (int i = 0; i < 8; i++) {
         data.lightSensorsData[i] = (uint8_t)(lineSensors[i]->read());
         data.distanceSensorsData[i] = (uint16_t)(distanceSensors[i]->readDistance());
    }

    data.IMUData = -2;
    return CommunicationHandler::encodeSensorsMessage(data);
}

EncodedMessage handleMotorsMessage(char *data) {
    MotorsData motorsData = CommunicationHandler::decodeMotorsMessage(data);

    for (int i = 0; i < 4; i++) {

        motors[i]->motorWrite(motorsData.motorsSpeeds[i]);
    }

    EncodedMessage message{};
    message.messageLength = 3;
    message.message = new char[message.messageLength];
    message.message[0] = MESSAGE_START;
    message.message[1] = MESSAGE_TYPE_MOTORS;
    message.message[2] = MESSAGE_END;

    return message;
}

EncodedMessage handleMotorsBrakeMessage() {
    motors[0]->brake();
    motors[1]->brake();
    motors[2]->brake();
    motors[3]->brake();

    EncodedMessage message{};
    message.messageLength = 3;
    message.message = new char[message.messageLength];
    message.message[0] = MESSAGE_START;
    message.message[1] = MESSAGE_TYPE_BRAKE_MOTORS;
    message.message[2] = MESSAGE_END;

    return message;
}

EncodedMessage handleEncodersMessage(const String& data) {
    if (data == "R") {
        resetEncoders = true;

        EncodedMessage message{};
        message.messageLength = 3;
        message.message = new char[message.messageLength];
        message.message[0] = MESSAGE_START;
        message.message[1] = MESSAGE_TYPE_ENCODERS;
        message.message[2] = MESSAGE_END;

        return message;
    }

    EncodersData encodersData{};
    for (int i = 0; i < 4; i++) {
        encodersData.encodersValues[i] = (int32_t)(encoders[i]->getDegrees());
    }

    return CommunicationHandler::encodeEncodersMessage(encodersData);
}

// Setup function.
void setup() {
    // Serial link setup.
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);

    Serial.begin(1000000);
    Wire.begin();

    for (int i = 0; i < 4; ++i) {
        motors[i] = new Motor(motorPins[i]);
    }

    for (int i = 0; i < 8; i++) {
        lineSensors[i] = new LightSensor(i);
    }

    for (int i = 0; i < 4; i++) {
        encoders[i] = new Encoder(0x08+i, 20, 30);
    }

    // Reset all distance sensors.
    for (auto distSensorSHTPin : distSensorsSHTPins) {
        pinMode(distSensorSHTPin, OUTPUT);
        digitalWrite(distSensorSHTPin, LOW);
    }
    delay(10);

    // Unreset all distance sensors.
    for (auto distSensorSHTPin : distSensorsSHTPins) {
        digitalWrite(distSensorSHTPin, HIGH);
    }
    delay(2);

    // Turn off all distance sensors.
    for (auto distSensorSHTPin : distSensorsSHTPins) {
        digitalWrite(distSensorSHTPin, LOW);
    }

    for (int i = 0; i < 8; i++) {
        digitalWrite(distSensorsSHTPins[i], HIGH);
        distanceSensors[i] = new DistanceSensor(DIST_SENSORS_BASE_ADDRESS+i, DIST_SENSORS_MEAS_PERIOD);
        DistanceSensorError err = distanceSensors[i]->begin();
        if (err != NoError) {
            if (err == AddrSetError) {
                Serial.print("Error setting i2c address for sensor ");
                Serial.println(i);
            } else if (err == MeasurementSetError) {
                Serial.print("Error setting continuous measurement for sensor ");
                Serial.println(i);
            } else {
                Serial.print("Unknown error, sensor ");
                Serial.println(i);
            }
        }
    }

//    mpu = new IMUSensor(imuInterruptPin);
//
//    digitalWrite(0, LOW);
//    digitalWrite(2, LOW);
//
//    mpu->init();
//    mpu->initDMP(63, 40, -20, -957, 2311,1710);
//
//    xTaskCreatePinnedToCore(
//            readMPU,
//            "IMU reading task",
//            100000,
//            nullptr,
//            0,
//            &readIMUSensor,
//            0);
}

// Loop function.
void loop() {
    if (Serial.available()) {
        // Read the request.
        char *message = nullptr;
        MessageType type = commHandler.readMessage(message);

        if (type != Error) {
            EncodedMessage respEncoded{};

            switch (type) {
            case SensorsType:
                respEncoded = handleDataMessage();
                break;
            case EncodersType:
                respEncoded = handleEncodersMessage(message);
                break;
            case MotorsType:
                respEncoded = handleMotorsMessage(message);
                break;
            case MotorsBrakeType:
                respEncoded = handleMotorsBrakeMessage();
                break;
            default:
                return;
            }

            Serial.write(respEncoded.message, respEncoded.messageLength);
            Serial.print("\n");
        }
    }
}
