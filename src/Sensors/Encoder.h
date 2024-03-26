#ifndef FIREFIGHTERSENSORINTERFACE_ENCODER_H
#define FIREFIGHTERSENSORINTERFACE_ENCODER_H

#include <Arduino.h>
#include <Wire.h>

class Encoder {
public:
    Encoder(int adress, int countsPerRevolution, int gearbox);

    int getDegrees();

    double getRotations() const;

    void reset() const;

private:
    int address;
    int countsPerRevolution;
    int gearbox;
};

#endif
