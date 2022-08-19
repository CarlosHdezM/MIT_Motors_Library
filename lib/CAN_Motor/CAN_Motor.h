#pragma once

#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>

class CanMotor{
    public:
        virtual bool initialize(const CAN_SPEED can_speed = CAN_1000KBPS, CAN_CLOCK can_clock = MCP_16MHZ) = 0;
};
