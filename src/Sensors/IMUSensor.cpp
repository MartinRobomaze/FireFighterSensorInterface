#include "IMUSensor.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>


volatile bool mpuInterrupt = false;

void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

IMUSensor::IMUSensor(int interruptPin) {
    IMUSensor::q = Quaternion();
    IMUSensor::gravity = VectorFloat();
    IMUSensor::imu = MPU6050();
    IMUSensor::mpuIntStatus = 0;
    IMUSensor::devStatus = 0;
    IMUSensor::packetSize = 0;
    IMUSensor::fifoCount = 0;

    pinMode(interruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReady, RISING);
    Wire.begin();
    Wire.setClock(400000);
}

bool IMUSensor::init() {
    IMUSensor::imu.initialize();
    return imu.testConnection();
}

bool IMUSensor::initDMP(short XGyroOffset, short YGyroOffset, short ZGyroOffset,
                        short XAccelOffset, short YAccelOffset, short ZAccelOffset) {
    devStatus = IMUSensor::imu.dmpInitialize();

    IMUSensor::imu.setXGyroOffset(XGyroOffset);
    IMUSensor::imu.setYGyroOffset(YGyroOffset);
    IMUSensor::imu.setZGyroOffset(ZGyroOffset);
    IMUSensor::imu.setXAccelOffset(XAccelOffset);
    IMUSensor::imu.setYAccelOffset(YAccelOffset);
    IMUSensor::imu.setZAccelOffset(ZAccelOffset);

    if (devStatus == 0) {
        IMUSensor::imu.CalibrateAccel(6);
        IMUSensor::imu.CalibrateGyro(6);

        IMUSensor::imu.setDMPEnabled(true);
        mpuIntStatus = IMUSensor::imu.getIntStatus();

        packetSize = IMUSensor::imu.dmpGetFIFOPacketSize();
    } else {
        return false;
    }

    return true;
}

void IMUSensor::readIMU() {
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
            fifoCount = IMUSensor::imu.getFIFOCount();
        }
    }

    mpuInterrupt = false;
    mpuIntStatus = IMUSensor::imu.getIntStatus();

    fifoCount = IMUSensor::imu.getFIFOCount();

    if (fifoCount < packetSize) {}
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        IMUSensor::imu.resetFIFO();

    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        while (fifoCount >= packetSize) {
            IMUSensor::imu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }

        IMUSensor::imu.dmpGetQuaternion(&q, fifoBuffer);
        IMUSensor::imu.dmpGetGravity(&gravity, &q);
        IMUSensor::imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}

float IMUSensor::getYawAngle() {
    return (float)(ypr[0] * 180 / PI);
}

float IMUSensor::getPitchAngle() {
    return (float)(ypr[1] * 180 / PI);
}

float IMUSensor::getRollAngle() {
    return (float)(ypr[2] * 180 / PI);
}