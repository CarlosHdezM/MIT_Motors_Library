#include "Encoder.h"
#include "Arduino.h"

// constructor
Encoder::Encoder(uint8_t _pinPWM, int _min_raw_count, int _max_raw_count)
{
    pinPWM = _pinPWM;

    cpr = _max_raw_count - _min_raw_count;
    min_raw_count = _min_raw_count;
    max_raw_count = _max_raw_count;

    // define if the sensor uses interrupts
    is_interrupt_based = false;

}


// initial hardware and variables 
void Encoder::init()
{

    pinMode(pinPWM, INPUT);

    // full rotations tracking number
    raw_count_prev = getRawCount();
    full_rotation_offset = 0;
}


// set current position as zero 
void Encoder::setZero()
{
    full_rotation_offset = -getAngle();
}



// get current angle (rad) 
float Encoder::getAngle()
{
    // raw data from encoder
    raw_count = getRawCount();

    int delta = raw_count - raw_count_prev;
    
    // if overflow happened track it as full rotation
    if (abs(delta) > (0.8 * cpr)) full_rotation_offset += delta > 0 ? -TWO_PI : TWO_PI;

    float angle = full_rotation_offset + ((float) (raw_count) / (float) cpr) * TWO_PI;

    // save variables for future pass
    raw_count_prev = raw_count;
    return angle;
}



// read the raw counter of the encoder
int Encoder::getRawCount()
{
    if (!is_interrupt_based)
    { // if it's not interrupt based read the value in a blocking way
        pulse_length_us = pulseIn(pinPWM, HIGH);
    }
    return pulse_length_us;
}



void Encoder::handlePWM()
{
    unsigned long now_us = micros();

    // if falling edge, calculate the pulse length
    if (!digitalRead(pinPWM)) pulse_length_us = now_us - last_call_us;

    // save the current timestamp for the next call
    last_call_us = now_us;
    is_interrupt_based = true; // set the flag to true
}



void Encoder::enableInterrupt(void (*doPWM)())
{
    // declare it's interrupt based
    is_interrupt_based = true;
    attachInterrupt(digitalPinToInterrupt(pinPWM), doPWM, CHANGE);
}