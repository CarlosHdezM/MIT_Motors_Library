#pragma once

#include "CAN_Motor.h"
#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>


class RmdMotor : public CanMotor{
    public:
        struct MotorType{
            const float reduction;
            const float KT;
            const float DIRECTION_SIGN;
            constexpr MotorType(float _reduction, float _kt, float _direction_sign) : reduction(_reduction), KT(_kt), DIRECTION_SIGN(_direction_sign){}  //Constructor
        };
        static const MotorType RMD_X6;
        static const MotorType RMD_X8_V1;
        static const MotorType RMD_X8_PRO_V1;
        //static const MotorType RMD_X8_V2;
        //static const MotorType RMD_X8_PRO_V2;
        static const MotorType RMD_X8_V3;
        static const MotorType RMD_X8_PRO_V3;
        static const MotorType RMD_L5015;

        RmdMotor(const MotorType & motor_type, const uint8_t _CS, const uint8_t _INT_PIN, const char * motor_name = "DEFAULT_RMD", SPIClass & spi = SPI, const bool doBegin = true);

        //Public member functions that this class defines (overrides from the base):
        bool turnOn() override;
        bool turnOff() override;
        bool setTorque(float torque_setpoint) override;
        bool setTorque(float torque_setpoint, unsigned long timeout_us) override;
        bool setCurrentPositionAsZero() override;
        bool setCurrentPositionAsOrigin() override;

        //Public member functions exlusive for RMD motors
        bool requestPosition();


    private:
        //Private member variables
        const MotorType m_motor_type;
        int8_t m_temperature;
        volatile bool m_curr_state = 0;

        //Private member functions that this class defines (overrides from the base):
        bool m_sendTorque(float torque_setpoint);
        bool m_readMotorResponse() override;

        //Private member functions exclusive for RMD Motors.
        bool m_requestPosition();
};