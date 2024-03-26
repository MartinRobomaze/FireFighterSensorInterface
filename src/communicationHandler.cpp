#include "communicationHandler.h"


DecodedMessage CommunicationHandler::readMessage() {
    // If message is valid.
    if (Serial.read() == MESSAGE_START) {
        int messageTypeChr = Serial.read();

        DecodedMessage message{};

        int bytesToRead;
        switch (messageTypeChr) {
            case MESSAGE_TYPE_SENSORS:
                message.type = SensorsType;
                bytesToRead = 0;
                break;
            case MESSAGE_TYPE_ENCODERS:
                message.type = EncodersType;
                bytesToRead = 0;
                break;
            case MESSAGE_TYPE_ENCODERS_RESET:
                message.type = EncodersResetType;
                bytesToRead = 0;
            case MESSAGE_TYPE_MOTORS:
                message.type = MotorsType;
                bytesToRead = 8;
                break;
            case MESSAGE_TYPE_BRAKE_MOTORS:
                message.type = MotorsBrakeType;
                bytesToRead = 0;
                break;
            default:
                return DecodedMessage{nullptr, Error};
        }

        if (bytesToRead != 0) {
            char *buffer = new char[bytesToRead];
            Serial.readBytes(buffer, bytesToRead);
            message.data = buffer;
        }

        // Read message end.
        Serial.read();

        return message;
    }

    // Invalid message.
    return DecodedMessage{nullptr, Error};
}

// Motors data:
// 2 bytes for motor speed (int16_t)
// 4 speeds are sent
MotorsData CommunicationHandler::decodeMotorsMessage(const char *data) {
    MotorsData motorsData{};

    for (int i = 0; i < 8; i += 2) {
        int16_t speed = (int16_t)((data[i] << 8) | data[i + 1]);
        motorsData.motorsSpeeds[(int) (i / 2)] = speed;
    }

    return motorsData;
}

// Sensors data:
// 1 byte for light sensor (uint8_t) (8 sensors)
// 2 bytes for distance sensors (uint16_t) (8 sensors)
// 4 bytes for IMU (int32_t)
EncodedMessage CommunicationHandler::encodeSensorsMessage(SensorsData data) {
    // 2 bytes - message start and message type
    // 8 bytes - light sensors data
    // 16 bytes distace sensors data
    // 4 bytes - imu data
    // 1 byte - message end
    EncodedMessage encodedMessage{};
    encodedMessage.messageLength = 2 + 8 + 16 + 4 + 1;
    encodedMessage.message = new char[encodedMessage.messageLength];

    int i = 0;

    encodedMessage.message[i] = MESSAGE_START;
    i++;
    encodedMessage.message[i] = MESSAGE_TYPE_SENSORS;
    i++;

    for (uint8_t lightSensorValue: data.lightSensorsData) {
        encodedMessage.message[i] = (char)lightSensorValue;
        i++;
    }

    for (uint16_t distanceSensorValue: data.distanceSensorsData) {
        encodedMessage.message[i] = (char)((distanceSensorValue >> 8) & 0xFF);
        i++;
        encodedMessage.message[i] = (char)(distanceSensorValue & 0xFF);
        i++;
    }

    encodedMessage.message[i] = (char)((data.IMUData >> 24) & 0xFF);
    i++;
    encodedMessage.message[i] = (char)((data.IMUData >> 16) & 0xFF);
    i++;
    encodedMessage.message[i] = (char)((data.IMUData >> 8) & 0xFF);
    i++;
    encodedMessage.message[i] = (char)((data.IMUData >> 0) & 0xFF);
    i++;

    encodedMessage.message[i] = MESSAGE_END;

    return encodedMessage;
}

// Encoders data:
// 4 bytes for encoders (int32_t) (4 encoders)
EncodedMessage CommunicationHandler::encodeEncodersMessage(EncodersData data) {
    // 2 bytes - message start and message type
    // 16 bytes - encoders data
    // 1 byte - message end
    EncodedMessage message{};
    message.messageLength = 2 + 16 + 1;
    message.message = new char[message.messageLength];

    int i = 0;

    message.message[i] = MESSAGE_START;
    i++;
    message.message[i] = MESSAGE_TYPE_SENSORS;
    i++;

    for (int32_t encoderValue: data.encodersValues) {
        message.message[i] = (char)((encoderValue >> 24) & 0xFF);
        i++;
        message.message[i] = (char)((encoderValue >> 16) & 0xFF);
        i++;
        message.message[i] = (char)((encoderValue >> 8) & 0xFF);
        i++;
        message.message[i] = (char)((encoderValue >> 0) & 0xFF);
        i++;
    }

    message.message[i] = MESSAGE_END;

    return message;
}
