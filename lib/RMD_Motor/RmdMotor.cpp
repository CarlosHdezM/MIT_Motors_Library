#include "RmdMotor.h"

RmdMotor::RmdMotor(const MotorType & motor_type, const uint8_t _CS,const char * motor_name, SPIClass & spi, const bool doBegin)
    : CanMotor{_CS, motor_name, spi, doBegin}, m_motor_type(motor_type) 
{
}