#include "MitMotor.h"

//Definition of static constants. 
const MitMotor::MotorType MitMotor::AK_10{-18.0f, 18.0f, 1.0f};
const MitMotor::MotorType MitMotor::GIM{-4.0f, 4.0f, 4.0f};
const float MitMotor::MotorType::P_MIN = -12.5f;
const float MitMotor::MotorType::P_MAX =  12.5f;
const float MitMotor::MotorType::V_MIN = -65.0f;
const float MitMotor::MotorType::V_MAX =  65.0f;
const float MitMotor::MotorType::KP_MIN = 0.0f;
const float MitMotor::MotorType::KP_MAX = 500.0f;
const float MitMotor::MotorType::KD_MIN = 0.0f;
const float MitMotor::MotorType::KD_MAX = 5.0f;


MitMotor::MitMotor(const MotorType & motor_type, const uint8_t _CS,const char * motor_name, SPIClass & spi, const bool doBegin)
    : m_motor_type(motor_type), name(motor_name), m_mcp2515{_CS, spi, doBegin}
{
}


bool MitMotor::initialize(const CAN_SPEED can_speed, CAN_CLOCK can_clock)
{
    #if DEBUG_ENABLED
        Serial.print("Initializing motor: "); Serial.println(name);
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


bool MitMotor::enterMotorMode(unsigned long timeout_us)
{
    constexpr uint8_t retry_times = 3;
    can_frame can_msg_sent;
    can_msg_sent.can_id  = 0x01;
    can_msg_sent.can_dlc = 0x08;
    can_msg_sent.data[0] = 0xFF;
    can_msg_sent.data[1] = 0xFF;
    can_msg_sent.data[2] = 0xFF;
    can_msg_sent.data[3] = 0xFF;
    can_msg_sent.data[4] = 0xFF;
    can_msg_sent.data[5] = 0xFF;
    can_msg_sent.data[6] = 0xFF;
    can_msg_sent.data[7] = 0xFC;
    for(uint i = 0; i < retry_times; i++){    
        m_mcp2515.sendMessage(&can_msg_sent);
        unsigned long t_ini = micros();
        while ( !readMotorResponse() and (micros()-t_ini) < timeout_us){}
        if ( !m_response_received){
            Serial.print("******************************\n");
            Serial.print(name); Serial.print(": NO ANSWER \n");
            Serial.print("******************************\n");
            return false;
        }
    }
    return true;
}



bool MitMotor::setCurrent(float current_setpoint ){
    can_frame can_msg;

    /// limit data to be within bounds ///
    float t_ff = constrain(current_setpoint, m_motor_type.T_MIN, m_motor_type.T_MAX);

    /// convert floats to unsigned ints ///
    unsigned int p_int  = m_float_to_uint(0, m_motor_type.P_MIN, m_motor_type.P_MAX, 16);
    unsigned int v_int  = m_float_to_uint(0, m_motor_type.V_MIN, m_motor_type.V_MAX, 12);
    unsigned int kp_int = m_float_to_uint(0, m_motor_type.KP_MIN, m_motor_type.KP_MAX, 12);
    unsigned int kd_int = m_float_to_uint(0, m_motor_type.KD_MIN, m_motor_type.KD_MAX, 12);
    unsigned int t_int  = m_float_to_uint(t_ff, m_motor_type.T_MIN, m_motor_type.T_MAX, 12);

    can_msg.can_id  = 0x01;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = p_int >> 8;;
    can_msg.data[1] = p_int & 0xFF;
    can_msg.data[2] = v_int >> 4;
    can_msg.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    can_msg.data[4] = kp_int & 0xFF;
    can_msg.data[5] = kd_int >> 4;
    can_msg.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    can_msg.data[7] = t_int & 0xFF;
    return m_mcp2515.sendMessage(&can_msg) == MCP2515::ERROR_OK ? true : false;
}



bool MitMotor::readMotorResponse()
{
    MCP2515::ERROR response_code = m_mcp2515.readMessage(&response_msg);
    if(response_code != MCP2515::ERROR::ERROR_OK)
    {
        m_response_received = false;
        return false;
    }
    /// unpack ints from can buffer ///
    unsigned int p_int_rx = (response_msg.data[1] << 8) | response_msg.data[2];
    unsigned int v_int_rx = (response_msg.data[3] << 4) | (response_msg.data[4] >> 4);
    unsigned int t_int_rx = ((response_msg.data[4] & 0xF) << 8) | response_msg.data[5];
    /// convert uints to floats ///
    m_position = (m_uint_to_float(p_int_rx, m_motor_type.P_MIN, m_motor_type.P_MAX, 16))/m_motor_type.P_DIVIDER;
    m_velocity = m_uint_to_float(v_int_rx, m_motor_type.V_MIN, m_motor_type.V_MAX, 12);
    m_torque = m_uint_to_float(t_int_rx, m_motor_type.T_MIN,  m_motor_type.T_MAX, 12);
    m_response_received = true;
    return true;
}



unsigned int MitMotor::m_float_to_uint(float x, float x_min, float x_max, int bits) 
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if (bits == 12) 
    {
        pgg = (unsigned int) ((x - offset) * 4095.0 / span);
    }
    if (bits == 16) 
    {
        pgg = (unsigned int) ((x - offset) * 65535.0 / span);
    }
    return pgg;
}


float MitMotor::m_uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) 
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    float pgg = 0;
    if (bits == 12) 
    {
        pgg = ((float)x_int) * span / 4095.0 + offset;
    }
    if (bits == 16) 
    {
        pgg = ((float)x_int) * span / 65535.0 + offset;
    }
    return pgg;
}
