#ifndef FIREFIGHTERSENSORINTERFACE_MCP23017_H
#define FIREFIGHTERSENSORINTERFACE_MCP23017_H

#include "I2Cdev.h"

#define GP0 0x00
#define GP1 0x01
#define IODIR0 0x06
#define IODIR1 0x07
#define INPUT_VAL 1
#define OUTPUT_VAL 0

class MCP23017 {
public:
    MCP23017();
    MCP23017(uint8_t devAddr);
    bool pinMode(uint8_t pin, uint8_t mode) const;
    bool write(uint8_t pin, uint8_t value) const;
    bool read(uint8_t pin, uint8_t *value) const;
private:
    uint8_t i2cAddr = 0x20;
};


#endif //FIREFIGHTERSENSORINTERFACE_MCP23017_H
