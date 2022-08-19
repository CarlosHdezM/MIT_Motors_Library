#pragma once

#include "CAN_Motor.h"
#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>

#define DEBUG_ENABLED 1

class MitMotor : public CanMotor{
    public:
        struct MotorType{
            const float T_MIN;
            const float T_MAX;
            const float P_DIVIDER;
            static const float P_MIN;
            static const float P_MAX;
            static const float V_MIN;
            static const float V_MAX;
            static const float KP_MIN;
            static const float KP_MAX;
            static const float KD_MIN;
            static const float KD_MAX;
            constexpr MotorType(float t_min, float t_max, float p_div) : T_MIN(t_min), T_MAX(t_max), P_DIVIDER(p_div){}
        };
        static const MotorType AK_10;
        static const MotorType GIM;

        //Public member variables
        can_frame response_msg;     //Tal vez convenga hacerla privada en el futuro.

        //Public member functions
        MitMotor(const MotorType & motor_type, const uint8_t _CS, const char * motor_name = "DEFAULT_NAME", SPIClass & spi = SPI, const bool doBegin = true);
        bool initialize(const CAN_SPEED can_speed = CAN_1000KBPS, CAN_CLOCK can_clock = MCP_16MHZ);
        bool enterMotorMode(unsigned long timeout_us = 5000);
        void exitMotorMode();
        void setCurrentPositionAsZero();
        bool setCurrent(float current_setpoint);
        bool readMotorResponse();
        MCP2515::ERROR cmd_cero_ak();
        const MotorType m_motor_type; //Public for debugging purposes, change to private. 
        const char * name;
        //Public "getters" to motor response variables. 
        const bool & wasResponseReceived() const {return m_response_received;}
        const float & position() const {return m_position;}
        const float & velocity() const {return m_velocity;}
        const float & torque() const {return m_torque;}

    private:
        //Private member variables
        MCP2515 m_mcp2515;
        float m_position;
        float m_velocity;
        float m_torque;
        bool m_response_received;

        //Private member functions
        void m_updateMotorResponseVariables();
        unsigned int m_float_to_uint(float x, float x_min, float x_max, int bits);
        float m_uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);

};