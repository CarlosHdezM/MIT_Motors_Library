#include "RmdMotor.h"

//Commands
#define SET_TORQUE_COMMAND    0xA1
#define REQUEST_POS_COMMAND   0x92
#define SET_ZERO_POS_COMMAND  0x19
#define TURN_OFF_COMMAND      0X80
#define UPDATE_STATUS_COMMAND 0x9C
//Motors Parameters
#define REDUCTION_1_TO_1      1.00f
#define REDUCTION_6_TO_1      6.00f
#define REDUCTION_9_TO_1      9.00f
#define L5015_KT              0.08f
#define X6_KT                 0.88f
#define X8_PRO_V1_KT          3.30f
#define X8_V1_KT              3.00f
#define X8_PRO_V2_KT          2.60f
#define X8_V2_KT              2.09f
#define X8_PRO_V3_KT          0.29f
#define X8_V3_KT              0.30f
#define CURRENT_RAW_MIN       -2000
#define CURRENT_RAW_MAX       2000
//Conversion Constants
#define AMPS_TO_RAW           62.5f           //Constant to convert from Amperes to RAW (in the motor manufacturer range).
#define RAD                   0.01745329251   //(pi/180)grad to rad 


//Definition of static constants. 
const RmdMotor::MotorType RmdMotor::RMD_X6{REDUCTION_6_TO_1, X6_KT};
const RmdMotor::MotorType RmdMotor::RMD_X8_V1{REDUCTION_6_TO_1, X8_V1_KT};
const RmdMotor::MotorType RmdMotor::RMD_X8_PRO_V1{REDUCTION_6_TO_1, X8_PRO_V1_KT};
const RmdMotor::MotorType RmdMotor::RMD_X8_V2{REDUCTION_9_TO_1, X8_V2_KT};
const RmdMotor::MotorType RmdMotor::RMD_X8_PRO_V2{REDUCTION_9_TO_1, X8_PRO_V2_KT};
const RmdMotor::MotorType RmdMotor::RMD_X8_V3{REDUCTION_6_TO_1, X8_V3_KT};
const RmdMotor::MotorType RmdMotor::RMD_X8_PRO_V3{REDUCTION_6_TO_1, X8_PRO_V3_KT};
const RmdMotor::MotorType RmdMotor::RMD_L5015{REDUCTION_1_TO_1, L5015_KT};



union torque_msg_tx
{
    struct
    {
        uint8_t command;
        int8_t null_1;
        int16_t null_2;
        int16_t torque;
        uint16_t null_3;
    } values;

    struct
    {
        uint8_t b0;
        uint8_t b1;
        uint8_t b2;
        uint8_t b3;
        uint8_t b4;
        uint8_t b5;
        uint8_t b6;
        uint8_t b7;
    } bytes;
};



union torque_msg_rx
{
    struct
    {
        uint8_t b0;
        uint8_t b1;
        uint8_t b2;
        uint8_t b3;
        uint8_t b4;
        uint8_t b5;
        uint8_t b6;
        uint8_t b7;
    } bytes;

    struct
    {
        uint8_t command;
        uint8_t temperature;
        int16_t torque;
        int16_t velocity;
        int16_t position_single_encoder_turn;
    } values;
};



RmdMotor::RmdMotor(const MotorType & motor_type, const uint8_t _CS, const uint8_t _INT_PIN, const char * motor_name, SPIClass & spi, const bool doBegin)
    : CanMotor{_CS, _INT_PIN, motor_name, spi, doBegin}, m_motor_type(motor_type)
{
}



bool RmdMotor::turnOn()
{
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = UPDATE_STATUS_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    
    if (!m_sendAndReceiveBlocking(can_msg, 1000000)) return false;

    can_msg.data[0] = REQUEST_POS_COMMAND;
    return m_sendAndReceiveBlocking(can_msg, 1000000);
}



bool RmdMotor::turnOff() 
{
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = TURN_OFF_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    if (!m_sendAndReceiveBlocking(can_msg, 1000000)) return false;

    can_msg.data[0] = REQUEST_POS_COMMAND;
    return m_sendAndReceiveBlocking(can_msg, 1000000);
}



//Refactor this function.
bool RmdMotor::setTorque(float torque_setpoint )
{
    if(m_is_auto_mode_running)
    {
        if (m_is_ready_to_send)
        {
            m_is_ready_to_send = false;
            if (m_curr_state == 0)
            {
                //Serial.println("Sending torque");
                m_curr_state = 1;
                return m_sendTorque(torque_setpoint);
            }
            else
            {
                //Serial.println("Requesting position");
                m_curr_state = 0;
                return m_requestPosition();
            }
        }
        else if ((millis() - m_last_response_time_ms) < MILLIS_LIMIT_UNTIL_RETRY) //In auto mode, test if we received response 
        {
            //Serial.println("All Ok, auto mode running normally");
            return true; //Everything OK. Motor doesn't need communication "recovery"
        }
        else
        {
            //Serial.print("\t Millis: "); Serial.print(millis()); Serial.print("\tLast message"); Serial.print(m_last_response_time_ms); Serial.print("\tRetrying to recover "); Serial.println(m_name); 
            cli();
            m_emptyMCP2515buffer();
            // m_mcp2515.clearRXnOVRFlags();
            // m_mcp2515.clearERRIF();
            // m_mcp2515.clearMERR();
            //m_mcp2515.clearInterrupts();
            sei();
            //m_last_response_time_ms = millis();
            if (m_curr_state == 0)
            {
                //Serial.println("Sending torque");
                m_curr_state = 1;
                m_sendTorque(torque_setpoint);
            }
            else
            {
                //Serial.println("Requesting position");
                m_curr_state = 0;
                m_requestPosition();
            }
            return false;
        }
    }
    return m_sendTorque(torque_setpoint);
}



bool RmdMotor::setTorque(float torque_setpoint, unsigned long timeout_us){
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



bool RmdMotor::setCurrentPositionAsZero()
{
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = SET_ZERO_POS_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    if (!m_sendAndReceiveBlocking(can_msg, 1000000)) return false;
    can_msg.data[0] = REQUEST_POS_COMMAND;
    if (!m_sendAndReceiveBlocking(can_msg, 1000000)) return false;
    m_offset_from_zero_motor = m_position;
    return true;
}



bool RmdMotor::setCurrentPositionAsOrigin(){
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = REQUEST_POS_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    //actualizar m_offset... con el nuevo valor de posición. 
    if (! m_sendAndReceiveBlocking(can_msg, 1000000))
    {
        return false;
    }
    m_offset_from_zero_motor = m_position;
    return true;
}



void RmdMotor::handleInterrupt(void)
{
    m_last_response_time_ms = millis();
    uint8_t irq = m_mcp2515.getInterrupts();
    //Serial.println(irq,BIN);
    if (irq & MCP2515::CANINTF_MERRF)
    {
        Serial.print("\n\n!!!!!ERROR MERF (ERROR IN MESSAGE TRANSMISSION OR RECEPTION)!!!"); Serial.print(m_name); Serial.print("\n\n");
        cli();
        m_mcp2515.clearMERR();
        m_mcp2515.clearInterrupts();
        sei();
    }
    if (irq & MCP2515::CANINTF_ERRIF)
    {
        //uint8_t err = m_mcp2515.getErrorFlags();
        Serial.print("\n\n!!!!!!!ERROR BUFFER FULL!!!!!!"); Serial.print(m_name); Serial.print("\n\n");
        //m_emptyMCP2515buffer();
        cli();
        m_mcp2515.clearRXnOVRFlags();
        m_mcp2515.clearERRIF();
        m_mcp2515.clearInterrupts();
        sei();
    }
    m_readMotorResponse();
    m_is_ready_to_send = true;
}



bool RmdMotor::requestPosition()
{
    //return (m_is_auto_mode_running ? true : m_requestPosition());
    if(m_is_auto_mode_running)
    {
        if ((millis() - m_last_response_time_ms) < MILLIS_LIMIT_UNTIL_RETRY) //In auto mode, test if we received response 
        {
            //Serial.println("All Ok, auto mode running normally");
            return true; //Everything OK. Motor doesn't need communication "recovery"
        }
        else
        {
            //Serial.print("Retrying to recover requestPosition "); Serial.println(m_name);
            cli();
            m_emptyMCP2515buffer();
            // m_mcp2515.clearRXnOVRFlags();
            // m_mcp2515.clearERRIF();
            // m_mcp2515.clearMERR();
            //m_mcp2515.clearInterrupts();
            sei();
            //m_last_response_time_ms = millis();
            m_requestPosition();
            return false;
        }
    }
    return m_requestPosition();
}



bool RmdMotor::m_sendTorque(float torque_setpoint)
{
    
    can_frame can_msg;
    torque_msg_tx msg_tx;
    msg_tx.values.torque = (int16_t)(((torque_setpoint)/m_motor_type.KT)*AMPS_TO_RAW);
    msg_tx.values.torque = constrain(msg_tx.values.torque, CURRENT_RAW_MIN, CURRENT_RAW_MAX);
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = SET_TORQUE_COMMAND;
    can_msg.data[1] = msg_tx.bytes.b1;
    can_msg.data[2] = msg_tx.bytes.b2;
    can_msg.data[3] = msg_tx.bytes.b3;
    can_msg.data[4] = msg_tx.bytes.b4;
    can_msg.data[5] = msg_tx.bytes.b5;
    can_msg.data[6] = msg_tx.bytes.b6;
    can_msg.data[7] = msg_tx.bytes.b7;
    //m_mcp2515.clearInterrupts();
    cli();
    bool result = (m_mcp2515.sendMessage(&can_msg) == MCP2515::ERROR_OK);
    sei();
    return result;
}



bool RmdMotor::m_readMotorResponse()
{
torque_msg_rx msg_rx;
    cli();
    if(m_mcp2515.readMessage(&response_msg) != MCP2515::ERROR::ERROR_OK)
    { 
        sei();
        return false;
    }
    sei();
    /// unpack ints from can buffer ///
    switch (response_msg.data[0])
    {
        case SET_TORQUE_COMMAND:
        case UPDATE_STATUS_COMMAND:
            msg_rx.bytes.b0 = response_msg.data[0];
            msg_rx.bytes.b1 = response_msg.data[1];
            msg_rx.bytes.b2 = response_msg.data[2];
            msg_rx.bytes.b3 = response_msg.data[3];
            msg_rx.bytes.b4 = response_msg.data[4];
            msg_rx.bytes.b5 = response_msg.data[5];
            msg_rx.bytes.b6 = response_msg.data[6];
            msg_rx.bytes.b7 = response_msg.data[7];
            m_temperature = msg_rx.values.temperature;
            m_torque = msg_rx.values.torque;
            m_velocity = msg_rx.values.velocity;
            //encoder(msg_rx.values.pos, values);;
            break;

        case REQUEST_POS_COMMAND:
            m_position = (((((response_msg.data[4] << 24) | (response_msg.data[3] << 16) | (response_msg.data[2] << 8) | (response_msg.data[1])))/(m_motor_type.reduction * 100.0))/** RAD*/);
            break;
            
        case SET_ZERO_POS_COMMAND:
            //m_position = 0;
            Serial.print("Recibida confirmación de seteo de cero. Motor "); Serial.println(m_name); Serial.println("!!!!NECESARIO REINICIAR ALIMENTACIÓN DEL MOTOR PARA QUE SEA VALIDO EL NUEVO CERO Y NO EXPLOTE ALV!!!");
            break;

        case TURN_OFF_COMMAND:
            Serial.print("Recibida confirmación de apagado. Motor: "); Serial.println(m_name);
            break;

        default:
            Serial.print("Se recibió una respuesta, pero no se reconoció el comando. Motor: "); Serial.println(m_name);
            return false;
            break;
    }
    return true;   
}



bool RmdMotor::m_requestPosition()
{
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = REQUEST_POS_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    //m_mcp2515.clearInterrupts();
    cli();
    bool result = m_mcp2515.sendMessage(&can_msg) == MCP2515::ERROR_OK;
    sei();
    return result;
}
