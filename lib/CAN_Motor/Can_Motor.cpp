#include "CAN_Motor.h"





CanMotor::CanMotor(const uint8_t _CS,const char * motor_name, SPIClass & spi, const bool doBegin)
    : m_name(motor_name), m_mcp2515{_CS, spi, doBegin}
{
}





bool CanMotor::initialize(const CAN_SPEED can_speed, CAN_CLOCK can_clock)
{
    #if DEBUG_ENABLED
        Serial.print("Initializing motor: "); Serial.println(m_name);
    #endif
    if ( m_mcp2515.reset() != MCP2515::ERROR_OK)
    {
        #if DEBUG_ENABLED
            Serial.println("!!!Error Resetting MCP2515.");
        #endif
        return false;
    }
    if ( m_mcp2515.setBitrate(can_speed, can_clock) != MCP2515::ERROR_OK)
    {
        #if DEBUG_ENABLED
            Serial.println("!!!Error Setting MCP2515 Bitrate.");
        #endif
        return false;
    }
    if ( m_mcp2515.setNormalMode() != MCP2515::ERROR_OK)
    {
        #if DEBUG_ENABLED
            Serial.println("!!!Error Setting MCP2515 normal mode.");
        #endif
        return false;
    }
    return true;
}



bool CanMotor::m_sendAndReceiveBlocking(const can_frame & can_msg , unsigned long timeout_us)
{
    MCP2515::ERROR response_code;
    unsigned long t_ini = micros();
    while ( (response_code = m_mcp2515.sendMessage(&can_msg)) != MCP2515::ERROR_OK and (micros()-t_ini) < timeout_us)
    {
        //Serial.println("Waiting free buffer to send command "); Serial.println(command_byte);
    }
    if ( response_code != MCP2515::ERROR_OK){
        #if DEBUG_ENABLED
            Serial.print("Failed sending message to "); Serial.println(m_name);
        #endif
        return false;
    }

    t_ini = micros();
    bool was_response_received;
    while ( !(was_response_received = readMotorResponse()) and (micros()-t_ini) < timeout_us)
    {
        /*Serial.println("Waiting Response");*/
    }
    if ( !was_response_received){
        Serial.print(m_name); Serial.print(": NO ANSWER \n");
        return false;
    }
    else 
    { 
        //Serial.print("Success time: "); Serial.println(i+1);
    }
    return true;    
}
