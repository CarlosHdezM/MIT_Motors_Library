#include "CAN_Motor.h"


CanMotor::CanMotor(const uint8_t _CS,const char * motor_name, SPIClass & spi, const bool doBegin)
    : m_interrupt_pin(99), m_name(motor_name), m_mcp2515{_CS, spi, doBegin}, m_torque(0), m_position(0), m_velocity(0), m_offset_from_zero_motor(0)
{
}


CanMotor::CanMotor(const uint8_t _CS, const uint8_t _INT_PIN, const char * motor_name, SPIClass & spi, const bool doBegin)
    : m_interrupt_pin(_INT_PIN), m_name(motor_name), m_mcp2515{_CS, spi, doBegin}, m_torque(0), m_position(0), m_velocity(0), m_offset_from_zero_motor(0)
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


/*
void CanMotor::irqHandler() 
{
    Serial.println("\nINTERRUPT RECEIVED\n");
    //readMotorResponse();
}
*/


/*
void CanMotor::startInterrupt(void (*ISR_callback)(void)){
    attachInterrupt(m_interrupt_pin, [](){
        Serial.println("\nINTERRUPT RECEIVED IN CLASS\n");
        //Serial.println(this->m_position);
    }
    , FALLING);  
}
*/

void CanMotor::handleInterrupt(void)
{
    Serial.println("\n\n!!!Received Interruptuc!!!\n\n");
    //Serial.print("My name is: "); Serial.println(m_name);
    readMotorResponse();
    setTorque(0);
}



void CanMotor::startInterrupt(void (*ISR_callback)(void)){
    attachInterrupt(m_interrupt_pin, ISR_callback, FALLING);  
}

/*
void CanMotor::startInterrupt(){
    attachInterrupt(m_interrupt_pin, std::bind(&CanMotor::irqHandler,this), FALLING);  
}
*/


bool CanMotor::m_sendAndReceiveBlocking(const can_frame & can_msg , unsigned long timeout_us)
{
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
    while(m_mcp2515.readMessage(&devnull) == MCP2515::ERROR_OK){ /*Serial.println("Vaciando buffer..."); */}
    return;
}
