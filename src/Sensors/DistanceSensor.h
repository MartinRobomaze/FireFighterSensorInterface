#include <Arduino.h>
#include "IO/mcp23017.h"


class DistanceSensor {
  public:
    DistanceSensor(uint8_t pingPin);

    uint8_t readDistance() const;
  private:
    uint8_t pingPin;
};