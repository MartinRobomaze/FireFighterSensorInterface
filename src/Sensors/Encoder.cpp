#include "Encoder.h"

Encoder::Encoder(int address, int countsPerRevolution, int gearbox) {
    Encoder::address = address;
    Encoder::countsPerRevolution = countsPerRevolution;
    Encoder::gearbox = gearbox;
}

int Encoder::getDegrees() {
    return int(Encoder::getRotations() * 360);
}

double Encoder::getRotations() const {
    Wire.requestFrom(Encoder::address, 4);
    byte buffer[4];

    for (int i = 0; Wire.available(); i++) {
        buffer[i] = Wire.read();
    }
    long value = buffer[3];
    value = value * 256 + buffer[2];
    value = value * 256 + buffer[1];
    value = value * 256 + buffer[0];
    return double(double(value) / double(Encoder::countsPerRevolution) / double(Encoder::gearbox));
}

void Encoder::reset() const {
    Wire.beginTransmission(Encoder::address);
    Wire.write("R");
    Wire.endTransmission();
}
