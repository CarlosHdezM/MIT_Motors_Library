#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"

// This encoder has been tested with AS5048a running in PWM mode.

class Encoder
{
public:
    //Encoder class constructor
    Encoder(uint8_t _pinPWM, int _min = 0, int _max = 903);//? value range for teensy 4.1 (4095 clock cycles / 903ms)

    // initialize the encoder hardware
    void init();

    int pinPWM;

    // set current position as zero
    void setZero();

    // get current angle (rad)
    float getAngle();
    
    // pwm handler
    void handlePWM();

    void enableInterrupt(void (*doPWM)());

    

private:
    // raw count
    int raw_count;
    int min_raw_count;
    int max_raw_count;
    int cpr;

    // time tracking variables
    unsigned long last_call_us;
    unsigned long pulse_length_us;

    // flag saying if the readings are interrupt based or not
    bool is_interrupt_based;

    // read the raw counter of the encoder
    int getRawCount();

    // total angle tracking variables
    float full_rotation_offset;
    int raw_count_prev;

};

#endif