#include "communicationHandler.h"


bool CommunicationHandler::decode(String message, int *messageType, String *data) {
    char messageCharArr[message.length()];

    // Creates char array from the message.
    message.toCharArray(messageCharArr, message.length());

    int i = 0;

    // If the start character is present.
    if (messageCharArr[i] == CommunicationHandler::messageStart) {
        i++;

        // If the message type is light sensor.
        if (messageCharArr[i] == CommunicationHandler::lightSensor) {
            i++;
            *data = CommunicationHandler::getDataFromMessage(message);
            *messageType = TYPE_LIGHT_SENSOR;
        }

            // If the message type is distance sensor.
        else if (messageCharArr[i] == CommunicationHandler::distanceSensor) {
            i++;
            *data = CommunicationHandler::getDataFromMessage(message);
            *messageType = TYPE_DISTANCE_SENSOR;
        }

            // If the message type is IMU sensor.
        else if (messageCharArr[i] == CommunicationHandler::imuSensor) {
            i++;
            *data = CommunicationHandler::getDataFromMessage(message);
            *messageType = TYPE_IMU;
        }

            // If the message type is motor.
        else if (messageCharArr[i] == CommunicationHandler::motor) {
            i++;
            *data = CommunicationHandler::getDataFromMessage(message);
            *messageType = TYPE_MOTOR;
        }

        else if (messageCharArr[i] == CommunicationHandler::echo) {
            *data = CommunicationHandler::getDataFromMessage(message);
            *messageType = TYPE_ECHO;
        }

        else if (messageCharArr[i] == CommunicationHandler::lightSensorsCalibration) {
            *data = CommunicationHandler::getDataFromMessage(message);
            *messageType = TYPE_LIGHT_SENSORS_CALIBRATION;
        }

        else if (messageCharArr[i] == CommunicationHandler::data) {
            *data = CommunicationHandler::getDataFromMessage(message);
            *messageType = TYPE_DATA;
        }

        else if (messageCharArr[i] == CommunicationHandler::motorsBrake) {
            *data = CommunicationHandler::getDataFromMessage(message);
            *messageType = TYPE_MOTORS_BRAKE;
        }

        else if (messageCharArr[i] == CommunicationHandler::encoder) {
            *data = CommunicationHandler::getDataFromMessage(message);
            *messageType = TYPE_ENCODER;
        }

            // Invalid message.
        else {
            return false;
        }
    }

        // Invalid message.
    else {
        return false;
    }

    // Valid message.
    return true;
}

String CommunicationHandler::encode(int messageType, String data) {
    String message = "";

    // Add tart character to the encoded message.
    message += CommunicationHandler::messageStart;

    // If the message type is light sensor.
    if (messageType == TYPE_LIGHT_SENSOR) {
        message += CommunicationHandler::lightSensor;
    }

        // If the message type is distance sensor.
    else if (messageType == TYPE_DISTANCE_SENSOR) {
        message += CommunicationHandler::distanceSensor;
    }

        // If the message type is IMU sensor.
    else if (messageType == TYPE_IMU) {
        message += CommunicationHandler::imuSensor;
    }

        // If the message type is motor.
    else if (messageType == TYPE_MOTOR) {
        message += CommunicationHandler::motor;
    }

    else if (messageType == TYPE_ECHO) {
        message += CommunicationHandler::echo;
    }

    else if (messageType == TYPE_LIGHT_SENSORS_CALIBRATION) {
        message += CommunicationHandler::lightSensorsCalibration;
    }

    else if (messageType == TYPE_MOTORS_BRAKE) {
        message += CommunicationHandler::motorsBrake;
    }

    else if (messageType == TYPE_MOTORS_BRAKE) {
        message += CommunicationHandler::motorsBrake;
    }

    else if (messageType == TYPE_ENCODER) {
        message += CommunicationHandler::encoder;
    }

        // Invalid message type.
    else {
        return "";
    }

    // Add data start character to message.
    message += CommunicationHandler::dataStart;
    // Add data to message.
    message += data;

    // Add data and message end to message.
    message += CommunicationHandler::dataEnd;
    message += CommunicationHandler::messageEnd;

    // Return encoded message.
    return message;
}

String CommunicationHandler::readMessage() {
    String message = "";
    // If message is valid.
    if (Serial.read() == CommunicationHandler::messageStart) {
        message += CommunicationHandler::messageStart;

        char reading = 'x';

        // While data are available and the message didn't end.
        while (Serial.available() && reading != CommunicationHandler::messageEnd) {
            reading = Serial.read();

            message += reading;
        }


        // Return the message.
        return message;
    }

    // Invalid message.
    return "";
}

String CommunicationHandler::getDataFromMessage(String message) {
    char messageCharArr[message.length()];

    // Convert the message to char array.
    message.toCharArray(messageCharArr, message.length());

    int i = 0;

    // While the data section isn't found
    while (messageCharArr[i] != '{') {
        i++;
    }
    i++;
    String data = "";

    // While the end of data isn't reached.
    while (messageCharArr[i] != '}') {
        data += messageCharArr[i];
        i++;
    }

    // Return data.
    return data;
}