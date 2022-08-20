#pragma once

#include "CAN_Motor.h"
#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>

#define DEBUG_ENABLED 1

class RmdMotor : public CanMotor{
    public:
        struct MotorType{
            //constexpr MotorType(float t_min, float t_max, float p_div) : T_MIN(t_min), T_MAX(t_max), P_DIVIDER(p_div){}  //Constructor
        };
        static const MotorType RMD_X6;
        static const MotorType RMD_X8;

        //Public member functions exlusive for RMD motors

        //Public member functions shared by all CAN motor types. 
        RmdMotor(const MotorType & motor_type, const uint8_t _CS, const char * motor_name = "DEFAULT_RMD", SPIClass & spi = SPI, const bool doBegin = true);
        // bool turnOn() override;
        // bool turnOff() override;
        // bool setCurrentPositionAsZero() override;
        // bool setCurrent(float current_setpoint) override;
        // bool setCurrent(float current_setpoint, unsigned long timeout_us) override;
        // bool readMotorResponse() override;
        // bool readMotorResponse(unsigned long timeout_us) override;


    private:
        //Private member variables
        const MotorType m_motor_type;


        //Private member functions


};