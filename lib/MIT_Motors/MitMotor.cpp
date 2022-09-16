#include "MitMotor.h"
#define TURN_ON_COMMAND  0XFC
#define TURN_OFF_COMMAND 0XFD
#define SET_ZERO_COMMAND 0XFE


//Definition of static constants. 
const MitMotor::MotorType MitMotor::AK_10{-18.0f, 18.0f, 1.0f, 1.0f};
const MitMotor::MotorType MitMotor::GIM{-4.0f, 4.0f, 2.0f, -1.0f};
const float MitMotor::MotorType::P_MIN = -12.5f;
const float MitMotor::MotorType::P_MAX =  12.5f;
const float MitMotor::MotorType::V_MIN = -65.0f;
const float MitMotor::MotorType::V_MAX =  65.0f;
const float MitMotor::MotorType::KP_MIN = 0.0f;
const float MitMotor::MotorType::KP_MAX = 500.0f;
const float MitMotor::MotorType::KD_MIN = 0.0f;
const float MitMotor::MotorType::KD_MAX = 5.0f;



MitMotor::MitMotor(const MotorType & motor_type, const uint8_t _CS, const uint8_t _INT_PIN, const char * motor_name, SPIClass & spi, const bool doBegin)
    : CanMotor{_CS, _INT_PIN, motor_name, spi, doBegin}, m_motor_type(motor_type)
{
}



bool MitMotor::turnOn()
{
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x01;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = 0xFF;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0xFF;
    can_msg.data[3] = 0xFF;
    can_msg.data[4] = 0xFF;
    can_msg.data[5] = 0xFF;
    can_msg.data[6] = 0xFF;
    can_msg.data[7] = TURN_ON_COMMAND;
    return m_sendAndReceiveBlocking(can_msg, 2000000);
}



bool MitMotor::turnOff()
{
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x01;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = 0xFF;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0xFF;
    can_msg.data[3] = 0xFF;
    can_msg.data[4] = 0xFF;
    can_msg.data[5] = 0xFF;
    can_msg.data[6] = 0xFF;
    can_msg.data[7] = TURN_OFF_COMMAND;
    return m_sendAndReceiveBlocking(can_msg, 2000000);
}



//Refactor this function.
bool MitMotor::setTorque(float torque_setpoint )
{
    //Normal mode (Polling based)
    if(!m_is_auto_mode_running) return m_sendTorque(torque_setpoint);


    //Auto mode (Interrupt driven)
    if (m_is_response_ready)
    {
        m_is_response_ready = false;
        uint8_t irq = m_mcp2515.getInterrupts();
        if (irq & MCP2515::CANINTF_MERRF)
        {
            Serial.print("\n\n!!!!!ERROR MERF (ERROR IN MESSAGE TRANSMISSION OR RECEPTION)!!!"); Serial.print(m_name); Serial.print("\n\n");
            m_mcp2515.clearMERR();
            m_mcp2515.clearInterrupts();
        }
        if (irq & MCP2515::CANINTF_ERRIF)
        {
            Serial.print("\n\n!!!!!!!ERROR BUFFER FULL!!!!!!"); Serial.print(m_name); Serial.print("\n\n");
            m_mcp2515.clearRXnOVRFlags();
            m_mcp2515.clearERRIF();
            m_mcp2515.clearInterrupts();
        }
        m_readMotorResponse();
        return m_sendTorque(torque_setpoint);
    }

    else if ((millis() - m_last_response_time_ms) < MILLIS_LIMIT_UNTIL_RETRY) //In case setTorque() was called again before message reception.
    {
        return true;
    }
    else
    {
        //Serial.print("\t Millis: "); Serial.print(millis()); Serial.print("\tLast message"); Serial.print(m_last_response_time_ms); Serial.print("\tRetrying to recover "); Serial.println(m_name); 
        if ((millis() - m_last_retry_time_ms) > MILLIS_LIMIT_UNTIL_RETRY)
        {
            m_last_retry_time_ms = millis();
            m_emptyMCP2515buffer();
            m_sendTorque(torque_setpoint);
        }
        return false;
    }
}



bool MitMotor::setTorque(float torque_setpoint, unsigned long timeout_us){
    if (m_is_auto_mode_running) 
    {
        return setTorque(torque_setpoint);
    }
    bool was_message_sent;
    unsigned long t_ini = micros();
    while(!(was_message_sent = setTorque(torque_setpoint)) and (micros()-t_ini) < timeout_us)
    {
        //Serial.println("Send Retry!");           
    }
    return was_message_sent;
}



bool MitMotor::setCurrentPositionAsZero()
{
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x01;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = 0xFF;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0xFF;
    can_msg.data[3] = 0xFF;
    can_msg.data[4] = 0xFF;
    can_msg.data[5] = 0xFF;
    can_msg.data[6] = 0xFF;
    can_msg.data[7] = SET_ZERO_COMMAND;
    if (!m_sendAndReceiveBlocking(can_msg, 2000000)) return false;
    return setCurrentPositionAsOrigin(); //This second command is used ALSO to get a correct new position value, since the SET_ZERO_COMMAND returns the motor position BEFORE the new zero was set.
}



bool MitMotor::setCurrentPositionAsOrigin(){
    if (!turnOn()) return false;
    m_offset_from_zero_motor = m_position;
    return true;
}



bool MitMotor::m_sendTorque(float torque_setpoint)
{
    can_frame can_msg;

    /// limit data to be within bounds ///
    float t_ff = constrain(torque_setpoint, m_motor_type.T_MIN, m_motor_type.T_MAX) * m_motor_type.DIRECTION_SIGN;
    unsigned int t_int  = m_float_to_uint(t_ff, m_motor_type.T_MIN, m_motor_type.T_MAX);

    can_msg.can_id  = 0x01;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = 0x7F;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0x7F;
    can_msg.data[3] = 0xf0;
    can_msg.data[4] = 0x0;
    can_msg.data[5] = 0x0;
    can_msg.data[6] = t_int >> 8;
    can_msg.data[7] = t_int & 0xFF;
    //m_mcp2515.clearInterrupts();
    bool result = (m_mcp2515.sendMessage(&can_msg) == MCP2515::ERROR_OK); 
    return result;
}



bool MitMotor::m_readMotorResponse(){
    MCP2515::ERROR response_code = m_mcp2515.readMessage(&response_msg);
    if(response_code != MCP2515::ERROR::ERROR_OK)
    {
        return false;
    }
    /// unpack ints from can buffer ///
    unsigned int p_int_rx = (response_msg.data[1] << 8) | response_msg.data[2];
    unsigned int v_int_rx = (response_msg.data[3] << 4) | (response_msg.data[4] >> 4);
    unsigned int t_int_rx = ((response_msg.data[4] & 0xF) << 8) | response_msg.data[5];
    /// convert uints to floats ///
    m_position = (m_uint_to_float(p_int_rx, m_motor_type.P_MIN, m_motor_type.P_MAX, 16)) / m_motor_type.P_DIVIDER * m_motor_type.DIRECTION_SIGN;
    m_velocity = m_uint_to_float(v_int_rx, m_motor_type.V_MIN, m_motor_type.V_MAX, 12) * m_motor_type.DIRECTION_SIGN;
    m_torque = m_uint_to_float(t_int_rx, m_motor_type.T_MIN,  m_motor_type.T_MAX, 12) * m_motor_type.DIRECTION_SIGN * m_motor_type.P_DIVIDER;
    return true;
}



unsigned int MitMotor::m_float_to_uint(float x, float x_min, float x_max) 
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (unsigned int) ((x - offset) * 4095.0 / span);
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
