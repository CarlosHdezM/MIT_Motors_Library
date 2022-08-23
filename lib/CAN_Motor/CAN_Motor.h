#pragma once

#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>

#define DEBUG_ENABLED 1


class CanMotor{
    public:
        CanMotor(const uint8_t _CS, const char * motor_name = "DEFAULT_NAME", SPIClass & spi = SPI, const bool doBegin = true);
        bool initialize(const CAN_SPEED can_speed = CAN_1000KBPS, CAN_CLOCK can_clock = MCP_16MHZ);
        virtual bool setCurrentPositionAsOrigin() = 0;
        virtual bool turnOn() = 0;
        virtual bool turnOff()= 0;
        virtual bool setCurrentPositionAsZero() = 0;
        virtual bool setCurrent(float current_setpoint) = 0;
        virtual bool setCurrent(float current_setpoint, unsigned long timeout_us) = 0;
        virtual bool readMotorResponse() = 0;
        virtual bool readMotorResponse(unsigned long timeout_us) = 0;
        //Public "getters" to motor response variables. 
        float position() const {return m_position - m_offset_from_zero_motor;}
        const float & velocity() const {return m_velocity;}
        const float & torque() const {return m_torque;}
        const char* name() const {return m_name;}

    protected:
        float m_position;
        float m_offset_from_zero_motor;
        float m_velocity;
        float m_torque;
        const char * m_name;
        can_frame response_msg;
        MCP2515 m_mcp2515;
        //Protected methods.
        bool m_sendAndReceiveBlocking(const can_frame & can_msg , unsigned long timeout_us);

};
