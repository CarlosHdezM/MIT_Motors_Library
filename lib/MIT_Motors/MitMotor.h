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

        //Public member functions common for all CAN motors:
        MitMotor(const MotorType & motor_type, const uint8_t _CS, const char * motor_name = "DEFAULT_NAME", SPIClass & spi = SPI, const bool doBegin = true);
        MitMotor(const MotorType & motor_type, const uint8_t _CS, const uint8_t _INT_PIN, const char * motor_name = "DEFAULT_NAME", SPIClass & spi = SPI, const bool doBegin = true);
        bool turnOn() override;
        bool turnOff() override;
        bool setCurrentPositionAsZero() override;
        bool setTorque(float torque_setpoint) override;
        bool setTorque(float torque_setpoint, unsigned long timeout_us) override;
        bool readMotorResponse() override;
        bool readMotorResponse(unsigned long timeout_us) override;
        bool setCurrentPositionAsOrigin() override;

        //Public member functions exclusive for MIT Motors.

    private:
        //Private member variables
        const MotorType m_motor_type;


        //Private member functions
        unsigned int m_float_to_uint(float x, float x_min, float x_max);
        float m_uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);

};