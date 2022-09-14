#include "CAN_Motor.h"



CanMotor::CanMotor(const uint8_t _CS, const uint8_t _INT_PIN, const char * motor_name, SPIClass & spi, const bool doBegin)
    : m_position(0), m_velocity(0), m_torque(0), m_name(motor_name), m_offset_from_zero_motor(0), m_interrupt_pin(_INT_PIN),
      m_is_auto_mode_running(false), m_last_response_time_ms(0), m_is_response_ready(true), m_mcp2515{_CS, spi, doBegin}
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



bool CanMotor::readMotorResponse()
{
    return (m_is_auto_mode_running ? true : m_readMotorResponse());
}



bool CanMotor::readMotorResponse(unsigned long timeout_us)
{
    if (m_is_auto_mode_running) return readMotorResponse();
    bool was_response_received;
    unsigned long t_ini = micros();
    while(!(was_response_received = readMotorResponse()) and (micros()-t_ini) < timeout_us)
    {
        //Serial.println("Waiting for response!");           
    }
    return was_response_received;
}



void CanMotor::startAutoMode(void (*ISR_callback)(void)){
    //Serial.print(m_name); Serial.println("Started auto mode"); 
    m_is_auto_mode_running = true;
    m_is_response_ready = true;
    m_emptyMCP2515buffer();
    m_mcp2515.clearInterrupts();
    pinMode(m_interrupt_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(m_interrupt_pin), ISR_callback, FALLING);
    m_last_response_time_ms = millis();
    return;
}



void CanMotor::stopAutoMode()
{
    m_is_auto_mode_running = false;
    detachInterrupt(digitalPinToInterrupt(m_interrupt_pin));
    m_emptyMCP2515buffer();
    m_mcp2515.clearInterrupts();
    return;
}



void CanMotor::handleInterrupt(void)
{
    m_last_response_time_ms = millis();
    m_is_response_ready = true;
}



bool CanMotor::m_sendAndReceiveBlocking(const can_frame & can_msg , unsigned long timeout_us)
{
    stopAutoMode();
    m_emptyMCP2515buffer();
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



void CanMotor::m_emptyMCP2515buffer()
{
    can_frame devnull;
    while(m_mcp2515.readMessage(&devnull) == MCP2515::ERROR_OK){ /*Serial.println("Vaciando buffer...");*/ }
    return;
}
