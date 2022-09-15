#pragma once

#include "CAN_Motor.h"
#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>


class MitMotor : public CanMotor{
    public:
        struct MotorType{
            const float T_MIN;
            const float T_MAX;
            const float P_DIVIDER;
            const float DIRECTION_SIGN;
            static const float P_MIN;
            static const float P_MAX;
            static const float V_MIN;
            static const float V_MAX;
            static const float KP_MIN;
            static const float KP_MAX;
            static const float KD_MIN;
            static const float KD_MAX;
            constexpr MotorType(float t_min, float t_max, float p_div, float direction_sign) : T_MIN(t_min), T_MAX(t_max), P_DIVIDER(p_div), DIRECTION_SIGN(direction_sign){}
        };
        static const MotorType AK_10;
        static const MotorType GIM;

        MitMotor(const MotorType & motor_type, const uint8_t _CS, const uint8_t _INT_PIN, const char * motor_name = "DEFAULT_NAME", SPIClass & spi = SPI, const bool doBegin = true);

        //Public member functions that this class defines (overrides from the base):
        bool turnOn() override;
        bool turnOff() override;
        bool setTorque(float torque_setpoint) override;
        bool setTorque(float torque_setpoint, unsigned long timeout_us) override;
        bool setCurrentPositionAsZero() override;
        bool setCurrentPositionAsOrigin() override;


    private:
        //Private member variables
        const MotorType m_motor_type;

        //Private member functions that this class defines (overrides from the base):
        bool m_sendTorque(float torque_setpoint) override;
        bool m_readMotorResponse() override;

        //Private member functions exclusive for MIT Motors.
        unsigned int m_float_to_uint(float x, float x_min, float x_max);
        float m_uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);
};