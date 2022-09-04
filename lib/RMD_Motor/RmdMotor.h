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
            constexpr MotorType(float _reduction, float _kt) : reduction(_reduction), KT(_kt){}  //Constructor
        };
        static const MotorType RMD_X6;
        static const MotorType RMD_X8_V1;
        static const MotorType RMD_X8_PRO_V1;
        static const MotorType RMD_X8_V2;
        static const MotorType RMD_X8_PRO_V2;
        static const MotorType RMD_X8_V3;
        static const MotorType RMD_X8_PRO_V3;
        static const MotorType RMD_L5015;

        //Public member functions exlusive for RMD motors
        bool requestPosition();

        //Public member functions shared by all CAN motor types. 
        RmdMotor(const MotorType & motor_type, const uint8_t _CS, const uint8_t _INT_PIN, const char * motor_name = "DEFAULT_RMD", SPIClass & spi = SPI, const bool doBegin = true);
        void handleInterrupt(void);
        bool turnOn() override;
        bool turnOff() override;
        bool setCurrentPositionAsZero() override;
        bool setCurrentPositionAsOrigin() override;

    private:
        //Private member variables
        const MotorType m_motor_type;
        uint8_t m_temperature;
        volatile bool m_curr_state = 0;

        bool m_sendTorque(float torque_setpoint);
        bool m_requestPosition();
        bool m_readMotorResponse() override;





};