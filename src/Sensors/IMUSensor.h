#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"


class IMUSensor {
  public:
    explicit IMUSensor(int interruptPin);
    void readIMU();
    float getYawAngle();
    float getPitchAngle();
    float getRollAngle();
    bool init();
    bool initDMP(short XGyroOffset, short YGyroOffset, short ZGyroOffset,
                 short XAccelOffset, short YAccelOffset, short ZAccelOffset);
  private:
    // [w, x, y, z] quaternion container.
    Quaternion q;
    // [x, y, z] gravity vector.
    VectorFloat gravity;
    // MPU6050 lib object.
    MPU6050 imu;
    // holds actual interrupt status byte from MPU.
    uint8_t mpuIntStatus;
    // return status after each device operation (0 = success, !0 = error).
    uint8_t devStatus;
    // expected DMP packet size (default is 42 bytes).
    uint16_t packetSize;
    // count of all bytes currently in FIFO.
    uint16_t fifoCount;
    // FIFO storage buffer.
    uint8_t fifoBuffer[64] = {0};
    // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector.
    float ypr[3] = {0.0, 0.0, 0.0};
};