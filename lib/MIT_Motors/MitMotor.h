#pragma once

#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>


class MitMotor {
    public:
        struct MotorType{
            const float T_MIN;
            const float T_MAX;
            constexpr MotorType(float t_min, float t_max) : T_MIN(t_min), T_MAX(t_max){}
        };
        static const MotorType AK_10;
        static const MotorType GIM;

        //Public member variables
        can_frame msg_response;

        //Public member functions
        MitMotor(const MotorType & motor_type, const uint8_t _CS, SPIClass & spi = SPI, const bool doBegin = true);
        void enterMotorMode();
        void exitMotorMode();
        void setCurrentPositionAsZero();
        MCP2515::ERROR setCurrent(float current_setpoint);
        MCP2515::ERROR read_motor_response();
        MCP2515::ERROR cmd_cero_ak();

        const MotorType m_motor_type; //Public for debugging purposes, change to private. 


    private:
        //MCP2515 m_mcp2515;
        float m_position;
        float m_velocity;
        float m_torque;
};