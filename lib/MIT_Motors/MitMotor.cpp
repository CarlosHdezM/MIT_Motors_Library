#include "MitMotor.h"

const MitMotor::MotorType MitMotor::AK_10{-18.0f, 18.0f};
const MitMotor::MotorType MitMotor::GIM{-4.0f, 4.0f};

MitMotor::MitMotor(const MotorType & motor_type, const uint8_t _CS, SPIClass & spi, const bool doBegin)
    : m_motor_type(motor_type){
}