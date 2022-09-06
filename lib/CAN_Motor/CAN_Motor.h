#pragma once

#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>

#define DEBUG_ENABLED 1
#define MILLIS_LIMIT_UNTIL_RETRY 50


class CanMotor{
    public:
        CanMotor(const uint8_t _CS, const uint8_t _INT_PIN, const char * motor_name = "DEFAULT_NAME", SPIClass & spi = SPI, const bool doBegin = true);

        bool initialize(const CAN_SPEED can_speed = CAN_1000KBPS, CAN_CLOCK can_clock = MCP_16MHZ);
        bool setTorque(float torque_setpoint);
        bool setTorque(float torque_setpoint, unsigned long timeout_us);
        bool readMotorResponse();
        bool readMotorResponse(unsigned long timeout_us);
        void startAutoMode(void (*ISR_callback)(void));
        void stopAutoMode();
        void (*ISR_callback)();

        virtual void handleInterrupt(void) = 0;        
        virtual bool setCurrentPositionAsOrigin() = 0;
        virtual bool turnOn() = 0;
        virtual bool turnOff()= 0;
        virtual bool setCurrentPositionAsZero() = 0;
        //Public "getters" to motor response variables. 
        float position() const {return m_position - m_offset_from_zero_motor;}
        const float & velocity() const {return m_velocity;}
        const float & torque() const {return m_torque;}
        const char* name() const {return m_name;}

        MCP2515 m_mcp2515;  //Temporarily put here for debugging purposes. MOVE TO PROTECTED.


    protected:
        float m_position;
        float m_offset_from_zero_motor;
        float m_velocity;
        float m_torque;
        const char * m_name;
        const uint8_t m_interrupt_pin;
        bool m_is_auto_mode_running = false;
        float m_torque_setpoint;
        unsigned long m_last_response_time_ms = 0;
        //unsigned long m_last_msg_sent_time_ms = 0;
        can_frame response_msg;
        //Protected methods.
        bool m_sendAndReceiveBlocking(const can_frame & can_msg , unsigned long timeout_us);
        void m_emptyMCP2515buffer();
        virtual bool m_sendTorque(float torque_setpoint) = 0;
        virtual bool m_readMotorResponse() = 0;

};
