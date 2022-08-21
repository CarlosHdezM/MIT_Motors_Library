#include "RmdMotor.h"

//Commands
#define SET_TORQUE_COMMAND    0xA1
#define REQUEST_POS_COMMAND   0x92
#define SET_ZERO_POS_COMMAND  0x19
#define TURN_OFF_COMMAND      0X80
#define UPDATE_STATUS_COMMAND 0x9C
//Conversiones
#define CONV_TORQUE 1.0f  //NO ES EL VALOR CORRECTO!, REVISAR CON CONTROL
#define CONV                  600.0f       //SOLO PARA LOS RMD X6
#define RAD                   0.0174533 //(pi/180)grad to rad 


//Definition of static constants. 
const RmdMotor::MotorType RmdMotor::RMD_X6{};
const RmdMotor::MotorType RmdMotor::RMD_X8{};


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


RmdMotor::RmdMotor(const MotorType & motor_type, const uint8_t _CS,const char * motor_name, SPIClass & spi, const bool doBegin)
    : CanMotor{_CS, motor_name, spi, doBegin}, m_motor_type(motor_type) 
{
}


bool RmdMotor::setCurrent(float current_setpoint)
{
    can_frame can_msg;
    torque_msg_tx msg_tx;
    msg_tx.values.torque = (current_setpoint * CONV_TORQUE);
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
    return (m_mcp2515.sendMessage(&can_msg) == MCP2515::ERROR_OK) ? true : false;
}


bool RmdMotor::readMotorResponse()
{
    return m_read_MCP_buffers();
}



bool RmdMotor::requestPosition()
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
    return (m_mcp2515.sendMessage(&can_msg) == MCP2515::ERROR_OK) ? true : false;
}



bool RmdMotor::m_read_MCP_buffers()
{
    torque_msg_rx msg_rx;  
    if(m_mcp2515.readMessage(&response_msg) != MCP2515::ERROR::ERROR_OK)
    { 
        return false;
    }
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
            m_position = ((((response_msg.data[4] << 24) | (response_msg.data[3] << 16) | (response_msg.data[2] << 8) | (response_msg.data[1])))/ CONV);//* RAD;
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


bool RmdMotor::turnOn(){
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
    return m_sendAndReceiveBlocking(can_msg, 1000000);
}



bool RmdMotor::setCurrentPositionAsZero()
{
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
    return m_sendAndReceiveBlocking(can_msg, 1000000);
}



bool RmdMotor::setCurrent(float current_setpoint, unsigned long timeout_us) { return true;}
bool RmdMotor::readMotorResponse(unsigned long timeout_us){return true;}