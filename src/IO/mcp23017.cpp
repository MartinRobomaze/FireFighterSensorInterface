//
// Created by martinrobomaze on 10. 11. 2021.
//

#include "mcp23017.h"


MCP23017::MCP23017() = default;

MCP23017::MCP23017(uint8_t devAddr) {
    MCP23017::i2cAddr = devAddr;
}

bool MCP23017::pinMode(uint8_t pin, uint8_t mode) const {
    uint8_t pin_values = 0;

    if (pin < 8) {
        bool success = I2Cdev::readByte(MCP23017::i2cAddr, IODIR0, &pin_values);
        if (!success) {
            return false;
        }

        if (mode == 1) {
            pin_values = pin_values | (mode >> pin % 8);
        } else if (mode == 0) {
            pin_values = pin_values & ~(mode >> pin % 8);
        }

        return I2Cdev::writeByte(MCP23017::i2cAddr, IODIR0, pin_values);
    } else {
        bool success = I2Cdev::readByte(MCP23017::i2cAddr, IODIR1, &pin_values);
        if (!success) {
            return false;
        }

        if (mode == 1) {
            pin_values = pin_values | (mode >> pin % 8);
        } else if (mode == 0) {
            pin_values = pin_values & ~(mode >> pin % 8);
        }

        return I2Cdev::writeByte(MCP23017::i2cAddr, IODIR1, pin_values);
    }
}

bool MCP23017::write(uint8_t pin, uint8_t value) const {
    uint8_t pin_values = 0;

    if (pin < 8) {
        bool success = I2Cdev::readByte(MCP23017::i2cAddr, GP0, &pin_values);
        if (!success) {
            return false;
        }

        if (value == 1) {
            pin_values = pin_values | (value >> pin % 8);
        } else if (value == 0) {
            pin_values = pin_values & ~(value >> pin % 8);
        }

        return I2Cdev::writeByte(MCP23017::i2cAddr, GP0, pin_values);
    } else {
        bool success = I2Cdev::readByte(MCP23017::i2cAddr, GP1, &pin_values);
        if (!success) {
            return false;
        }

        if (value == 1) {
            pin_values = pin_values | (value >> pin % 8);
        } else if (value == 0) {
            pin_values = pin_values & ~(value >> pin % 8);
        }

        return I2Cdev::writeByte(MCP23017::i2cAddr, GP1, pin_values);
    }
}

bool MCP23017::read(uint8_t pin, uint8_t *value) const {
    uint8_t pin_values = 0;

    if (pin < 8) {
        bool success = I2Cdev::readByte(MCP23017::i2cAddr, GP0, &pin_values);
        if (!success) {
            return false;
        }

        *value = (pin_values >> pin % 8) & 1;
    } else {
        bool success = I2Cdev::readByte(MCP23017::i2cAddr, GP1, &pin_values);
        if (!success) {
            return false;
        }

        *value = (pin_values >> pin % 8) & 1;
    }

    return true;
}
