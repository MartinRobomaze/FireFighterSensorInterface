#include <Arduino.h>
#include <Wire.h>
#include <QTRSensors.h>
#include <Motors/Motor.h>
#include <Sensors/DistanceSensor.h>
#include "Sensors/LightSensor.h"
#include <Sensors/IMUSensor.h>
#include <Sensors/Encoder.h>
#include <communicationHandler.h>


int motorPins[8] = {4, 16, 18, 19, 23, 13, 12, 27};
uint8_t lightSensorPins[8] = {35, 2, 34, 39, 32, 33, 25, 26};
int distSensorsPins[8][2] = {
        {0,  1},
        {2,  3},
        {4,  5},
        {6,  7},
        {8,  9},
        {10, 11},
        {12, 13},
        {14, 15}
};

int motorChannels[4][2] = {
        {0, 1},
        {2, 3},
        {4, 5},
        {6, 7}
};

int imuInterruptPin = 15;
bool resetEncoders = false;

Motor *motors;
LightSensor *lineSensors;
DistanceSensor *distanceSensors;
Encoder *encoders;

//IMUSensor *mpu;

double encoderValues[4];

// Creates communicationHandler class instance.
CommunicationHandler commHandler;

void handleDataMessage(String *encodedMessage) {
    uint16_t sensorValues[8];

    String lightSensorsData = "";
    String distanceSensorsData = "";
    String IMUSensorData = "";
    String encodersData = "";

    for (int i = 0; i < 8; i++) {
        lightSensorsData += String(lineSensors[i].read()) + ",";

        if (i < 4) {
            encodersData += String(encoders[i].getRotations());
        }

//        distanceSensorsData += String(distanceSensors[i].readDistance()) + ",";
        distanceSensorsData += "0,";
    }

//    IMUSensorData = String(mpu->getYawAngle());
    IMUSensorData = "0";

    lightSensorsData = lightSensorsData.substring(0, lightSensorsData.lastIndexOf(','));
    String lightSensorsDataEncoded = commHandler.encode(TYPE_LIGHT_SENSOR, lightSensorsData);

    distanceSensorsData = distanceSensorsData.substring(0, distanceSensorsData.lastIndexOf(','));
    String distanceSensorsDataEncoded = commHandler.encode(TYPE_DISTANCE_SENSOR, distanceSensorsData);

    String IMUSensorDataEncoded = commHandler.encode(TYPE_IMU, IMUSensorData);

    *encodedMessage = '~' + lightSensorsDataEncoded + '\t' + distanceSensorsDataEncoded + '\t' + IMUSensorDataEncoded;
}

void handleMotorsMessage(const String& data, String *encodedMessage) {
    // Get the motor to be turned on.
    String motor = data.substring(0, data.indexOf(","));
    int motorIndex;

    if (motor == "A") {
        motorIndex = 0;
    } else if (motor == "B") {
        motorIndex = 1;
    } else if (motor == "C") {
        motorIndex = 2;
    } else if (motor == "D") {
        motorIndex = 3;
    } else {
        return;
    }

    // Get the direction.
    char direction[10];
    data.substring(data.indexOf(",") + 1, data.lastIndexOf(",")).toCharArray(direction, 10);
    int speed = (int)data.substring(data.lastIndexOf(",") + 1, data.indexOf("}")).toInt();

    // Write the motor.
    motors[motorIndex].motorWrite(direction[0], speed);

    // Form the response.
    String response = motor + "," + direction[0] + "," + speed;

    // Encode the response.
    *encodedMessage = "~" + commHandler.encode(TYPE_MOTOR, response);
}

void handleMotorsBrakeMessage(const String& message, String *encodedMessage) {
    motors[0].brake();
    motors[1].brake();
    motors[2].brake();
    motors[3].brake();

    *encodedMessage = '~' + message;
}

void handleEncodersMessage(const String& data, const String& message, String *encodedMessage) {
    String encodersData;

    if (data == "R") {
        resetEncoders = true;

        *encodedMessage = "~" + message;
    }

    for (double encoderValue : encoderValues) {
        encodersData += String(encoderValue) + ",";
    }

    encodersData = encodersData.substring(0, encodersData.lastIndexOf(','));

    *encodedMessage = "~" + commHandler.encode(TYPE_ENCODER, encodersData);
}

TaskHandle_t readIMUSensor;

[[noreturn]] void readMPU(void *params) {
    (void)params;

    for (;;) {
//        mpu->readIMU();
        for (int i = 0; i < 4; i++) {
            if (resetEncoders) {
                encoders[i].reset();
            } else {
                if (i == 1 || i == 2) {
                    encoderValues[i] = -encoders[i].getRotations();
                } else {
                    encoderValues[i] = encoders[i].getRotations();
                }
            }
        }
        resetEncoders = false;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Setup function.
void setup() {
    // Serial link setup.
    Serial.begin(1000000);
    Wire.begin();

    motors = new Motor[4]{
            Motor(motorChannels[0]),
            Motor(motorChannels[1]),
            Motor(motorChannels[2]),
            Motor(motorChannels[3])
    };

    lineSensors = new LightSensor[8]{
        LightSensor(lightSensorPins[0]),
        LightSensor(lightSensorPins[1]),
        LightSensor(lightSensorPins[2]),
        LightSensor(lightSensorPins[3]),
        LightSensor(lightSensorPins[4]),
        LightSensor(lightSensorPins[5]),
        LightSensor(lightSensorPins[6]),
        LightSensor(lightSensorPins[7])
    };

    distanceSensors = new DistanceSensor[8]{
            DistanceSensor(0x20, distSensorsPins[0][0], distSensorsPins[0][1]),
            DistanceSensor(0x20, distSensorsPins[1][0], distSensorsPins[1][1]),
            DistanceSensor(0x20, distSensorsPins[2][0], distSensorsPins[2][1]),
            DistanceSensor(0x20, distSensorsPins[3][0], distSensorsPins[3][1]),
            DistanceSensor(0x20, distSensorsPins[4][0], distSensorsPins[4][1]),
            DistanceSensor(0x20, distSensorsPins[5][0], distSensorsPins[5][1]),
            DistanceSensor(0x20, distSensorsPins[6][0], distSensorsPins[6][1]),
            DistanceSensor(0x20, distSensorsPins[7][0], distSensorsPins[7][1])
    };

    encoders = new Encoder[4]{
            Encoder(0x08, 20, 30),
            Encoder(0x09, 20, 30),
            Encoder(0x0A, 20, 30),
            Encoder(0x0B, 20, 30)
    };

//    mpu = new IMUSensor(imuInterruptPin);

    // Motors pins setup.
    int k = 0;
    for (auto & motorChannel : motorChannels) {
        for (int j : motorChannel) {
            pinMode(motorPins[k], OUTPUT);
            ledcSetup(j, 1000, 8);
            ledcAttachPin(motorPins[k], j);
            k++;
        }
    }

//    digitalWrite(0, LOW);
//    digitalWrite(2, LOW);
//
//    mpu->init();
//    mpu->initDMP(63, 40, -20, -957, 2311,1710);

//    xTaskCreatePinnedToCore(
//            readMPU,
//            "IMU reading task",
//            100000,
//            nullptr,
//            0,
//            &readIMUSensor,
//            0);

    Serial.println(xPortGetCoreID());
}

// Loop function.
void loop() {
    if (Serial.available()) {
        // Read the request.
        String message = commHandler.readMessage();
        
        int messageType;
        String data;
        // Decode the request.
        bool isValid = commHandler.decode(message, &messageType, &data);

//      If the request is valid.
        if (isValid) {
            String response;
            String responseEncoded;

            if (messageType == TYPE_DATA) {
                handleDataMessage(&responseEncoded);
            } else if (messageType == TYPE_ENCODER) {
                handleEncodersMessage(data, message, &responseEncoded);
            } else if (messageType == TYPE_MOTOR) {
                handleMotorsMessage(data, &responseEncoded);
            } else if (messageType == TYPE_MOTORS_BRAKE) {
                handleMotorsBrakeMessage(message, &responseEncoded);
            }
            else {
                return;
            }

            // Send the response.
            responseEncoded += "\n";
            Serial.print(responseEncoded);
        }
    }
}
