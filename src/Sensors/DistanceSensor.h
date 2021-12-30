#include <Arduino.h>
#include "IO/mcp23017.h"


class DistanceSensor {
  public:
    DistanceSensor(uint8_t mcp23071Addr, uint8_t echoPin, uint8_t trigPin);

    uint8_t readDistance();
  private:
    uint8_t echoPin;
    uint8_t trigPin;
    MCP23017 expaderHandle;
};