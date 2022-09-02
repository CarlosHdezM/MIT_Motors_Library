#ifndef _MACHINE_STATE_H_
#define _MACHINE_STATE_H_

#include <Arduino.h>

enum MachineStates
{
    PRINT_MENU, 
    WAIT_SELECTION, 
    ENABLE_MOTORS, 
    DISABLE_MOTORS, 
    SET_TORQUE_ZERO,
    READ_MOTOR_RESPONSE,
    REQUEST_POSITION,
    SET_TORQUE,
    SET_TORQUE_AND_READ,
    SET_POS_ORIGIN, 
    SET_POS_ZERO
};


enum MachineSerialInputs : char 
{
    INPUT_NO_NEW_INPUT = '0',
    INPUT_ENABLE_MOTORS = '1',
    INPUT_DISABLE_MOTORS = '2',
    INPUT_SET_TORQUE_ZERO = '3',
    INPUT_READ_MOTOR_RESPONSE = '4',
    INPUT_REQUEST_POSITION = '5',
    INPUT_SET_TORQUE = '6',
    INPUT_SET_TORQUE_AND_READ = '7',
    INPUT_SET_POS_ORIGIN = '8',
    INPUT_SET_POS_ZERO  = '9'

};

void print_menu(void);

#endif