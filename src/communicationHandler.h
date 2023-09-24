#ifndef FIREFIGHTERSENSORINTERFACE_COMMUNICATIONHANDLER_H
#define FIREFIGHTERSENSORINTERFACE_COMMUNICATIONHANDLER_H


#include <Arduino.h>

#define MESSAGE_TYPE_MOTORS 'M'
#define MESSAGE_TYPE_SENSORS 'S'
#define MESSAGE_TYPE_ENCODERS 'E'
#define MESSAGE_TYPE_BRAKE_MOTORS 'B'

#define MESSAGE_START '<'
#define MESSAGE_END '>'

enum MessageType {
  SensorsType,
  EncodersType,
  MotorsType,
  MotorsBrakeType,
  Error
};

struct SensorsData {
  u_int8_t lightSensorsData[8];
  u_int16_t distanceSensorsData[8];
  int32_t IMUData;
};

struct MotorsData {
  int16_t motorsSpeeds[4];
};

struct EncodersData {
  int32_t encodersValues[4];
};

struct EncodedMessage {
  char *message;
  int messageLength;
};

class CommunicationHandler {
  public:
    /**
     * @brief Reads message via serial link.
     * @param data Data extracted from the message.
     * @returns Message type.
    */
    MessageType readMessage(char *data);
    
    /**
     * @brief Decodes data from message to control motors.
     * @param data Raw message data.
     * @returns Decoded motor data.
    */
    static MotorsData decodeMotorsMessage(const char *data);

    /**
     * @brief Encodes SensorsData to raw message.
     * @param data Sensors data.
     * @returns Raw message.
    */
    static EncodedMessage encodeSensorsMessage(SensorsData data);
    
    /**
     * @brief Encodes EncodersData to raw message.
     * @param data Encoders data.
     * @returns Raw message.
    */
    static EncodedMessage encodeEncodersMessage(EncodersData data);
  private:
};

#endif
