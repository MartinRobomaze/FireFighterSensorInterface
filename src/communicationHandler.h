/**
 * Handles communication between the interface and anothr device via serial link. It encodes, decodes and 
 * reads messages in this format:
 *      < message_type { data } >
 *    
 *    < - message start character 
 *    message_type:
 *      A - dumping all sensors data
 *      L - light sensor reading
 *      D - distance sensor reading
 *      I - IMU sensor reading(not implemented yet)
 *      M - motor writing
 *      E - echo - test connection
 * 
 *    { - data block start character
 *    data - data to send
 *    } - data block end character
 *    > - message end character
 */

#include <Arduino.h>
#define TYPE_LIGHT_SENSOR 0
#define TYPE_DISTANCE_SENSOR 1
#define TYPE_MOTOR 2
#define TYPE_IMU 3
#define TYPE_ECHO 4
#define TYPE_LIGHT_SENSORS_CALIBRATION 5
#define TYPE_DATA 6
#define TYPE_MOTORS_BRAKE 7
#define TYPE_ENCODER 8


class CommunicationHandler {
  public:
    /// <Summary> Reads message via serial link
    String readMessage();
    
    /// <Summary> Decodes message received from another device.
    /// <param name="message"> The received message
    /// <param name="*messageType"> Return value, the type of message.
    /// <param name="*data"> Return value, the data received.
    bool decode(String message, int *messageType, String *data);

    /// <Summary> Encodes message.
    /// <param name="messageType"> The type of the message to be encoded.
    /// <param name="data"> The data to be encoded.
    String encode(int messageType, String data);
  private:
    String getDataFromMessage(String message);
    
    const char lightSensor = 'L';
    const char distanceSensor = 'D';
    const char imuSensor = 'I';
    const char motor = 'M';
    const char echo = 'E';
    const char data = 'A';
    const char encoder = 'N';
    const char lightSensorsCalibration = 'C';
    const char motorsBrake = 'B';
    const char messageStart = '<';
    const char messageEnd = '>';
    const char dataStart = '{';
    const char dataEnd = '}';
};