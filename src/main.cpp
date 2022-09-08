#include "Arduino.h"
#include "machine_state.h"
#include "MitMotor.h"
#include "RmdMotor.h"
#include "array"
#include <algorithm>
 

// const uint8_t CS_PINS [NUM_MOTORS] =        {2 , 3 };
// const uint8_t INTERRUPT_PINS[NUM_MOTORS] =  {27, 28};
#define CS_1 2
#define CS_2 3
#define INT_1 27
#define INT_2 28

#define BOTON 1
#define AUX_PIN_1 9


MachineStates current_state = MachineStates::PRINT_MENU;


CanMotor * motors[] = {
    new MitMotor(MitMotor::AK_10, CS_1, INT_1, "AK_10 1"),
    new RmdMotor(RmdMotor::RMD_X6, CS_2, INT_2, "RMD X6 1" )
};
const size_t NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);


// std::array<CanMotor *, NUM_MOTORS> my_motors_std = {
//     new MitMotor(MitMotor::AK_10, CS_1, INT_1, "AK_10 1"),
//     new RmdMotor(RmdMotor::RMD_X6, CS_2, INT_2, "RMD X6 1" )
// };


void setup()
{
    Serial.begin(115200);
    pinMode(BOTON, INPUT_PULLUP);
    pinMode(AUX_PIN_1, OUTPUT);
    digitalWrite(AUX_PIN_1,LOW);
    delay(500);

    Serial.println("Sizes:");
    Serial.println(sizeof(uint8_t));
    Serial.println(sizeof(MitMotor));
    Serial.println(sizeof(RmdMotor));

    for (auto & motor : motors)
    {
        while(!motor->initialize())
        {
            Serial.print("Retrying to initialize "); Serial.print(motor->name()); Serial.print(" MCP2515");
        }
        while(!motor->turnOn())
        {
            Serial.print("Retrying to turn on "); Serial.println(motor->name());
        }
    }
    Serial.println("All motors initialized succesfully");

}


void loop ()
{
    MachineSerialInputs serial_input = INPUT_NO_NEW_INPUT;
    while(Serial.peek() == '\n' or Serial.peek() == '\r') Serial.read();
    if(Serial.available())
    {
        serial_input = (MachineSerialInputs)Serial.read();
    }
    switch (current_state)
    {
        case PRINT_MENU:
            print_menu();
            //State transition 
            current_state = MachineStates::WAIT_SELECTION;
            break;

        case WAIT_SELECTION:
            //This state only transitions states.
            switch (serial_input)
            {
                case INPUT_NO_NEW_INPUT:
                    current_state = WAIT_SELECTION;
                    break;

                case INPUT_ENABLE_MOTORS:
                    current_state = ENABLE_MOTORS;
                    break;

                case INPUT_DISABLE_MOTORS:
                    current_state = DISABLE_MOTORS;
                    break;

                case INPUT_SET_TORQUE_ZERO:
                    current_state = SET_TORQUE_ZERO;
                    break;

                case INPUT_READ_MOTOR_RESPONSE:
                    current_state = READ_MOTOR_RESPONSE;
                    break;

                case INPUT_REQUEST_POSITION:
                    current_state = REQUEST_POSITION; 
                    break;

                case INPUT_SET_TORQUE:
                    current_state = SET_TORQUE; 
                    break;

                case INPUT_SET_TORQUE_AND_READ:
                    current_state = SET_TORQUE_AND_READ; 
                    break;
                
                case INPUT_SET_POS_ORIGIN:
                    current_state = SET_POS_ORIGIN; 
                    break;

                case INPUT_SET_POS_ZERO:
                    current_state = SET_POS_ZERO; 
                    break;

                case INPUT_AUTO_MODE_ON:
                    current_state = START_AUTO_MODE;
                    break;

                case INPUT_AUTO_MODE_OFF:
                    current_state = STOP_AUTO_MODE;
                    break;

                default:
                    Serial.println("\nInvalid Option");
                    current_state = PRINT_MENU;
                    break;
            }
            break;

        case ENABLE_MOTORS:
            Serial.println("\nEnabling Motor");
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                if (motors[i]->turnOn() ) {Serial.print("Turned on "); Serial.println(motors[i]->name());}
                else {Serial.print("Failed turning on "); Serial.println(motors[i]->name());}
            }            
            current_state = MachineStates::PRINT_MENU;
            break;

        case DISABLE_MOTORS:
            Serial.println("\nDisabling Motor");   
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                if (motors[i]->turnOff() ) {Serial.print("Turned off "); Serial.println(motors[i]->name());}
                else {Serial.print("Failed turning off "); Serial.println(motors[i]->name());}
            }   
            current_state = MachineStates::PRINT_MENU;
            break;

        case SET_TORQUE_ZERO:
            Serial.print("\nEstableciendo el torque a 0\n");
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                if (motors[i]->setTorque(0)) {Serial.print("Current 0 setpoint sent to "); Serial.println(motors[i]->name());}
                else {Serial.print("Failed setting torque cero to "); Serial.println(motors[i]->name());}
            }            
            current_state = MachineStates::PRINT_MENU;
            break;

        case READ_MOTOR_RESPONSE:
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {       
                if (motors[i]->readMotorResponse())
                {
                    Serial.print("\nL Respuesta motores\n");
                    Serial.print("Position: "); Serial.print(motors[i]->position(), 4);
                    Serial.print("\tTorque: "); Serial.print(motors[i]->torque(), 4);
                    Serial.print("\tVelocity: "); Serial.println(motors[i]->velocity(), 4);
                }
                else {Serial.print("No response pending in "); Serial.print(motors[i]->name()); Serial.print(" MCP2515 Buffer");}
            }

            current_state = MachineStates::PRINT_MENU;
            break;

        case REQUEST_POSITION:
            Serial.println("Muchas gracias por seleccionar esta opcion, pero este caso no se usa para nada ahorita.");
            current_state = MachineStates::PRINT_MENU;
            break;

        case SET_TORQUE:
            Serial.println("Muchas gracias por seleccionar esta opcion, pero este caso no se usa para nada ahorita x2.");
            current_state = MachineStates::PRINT_MENU;
            break;

        case SET_TORQUE_AND_READ:
        {
            Serial.println("\nVamos a escribir y leer continuamente");
            elapsedMicros wait_to_print;
            elapsedMicros wait_to_loop;
            const uint16_t delay_loop_us = 0;
            const uint16_t print_every_us = 5000;
            while (digitalRead(BOTON) == HIGH)
            {
                while (wait_to_loop < delay_loop_us){}
                wait_to_loop = 0;
                
                for (uint8_t i = 0; i < NUM_MOTORS; i++)
                {  
                    if(!motors[i]->setTorque(0)) { /*Serial.print("No se pudo enviar torque 0 a: "); Serial.println(motors[i]->name());*/}
                }
                
                if(wait_to_print > print_every_us)
                {
                    wait_to_print = 0;
                    for (uint8_t i = 0; i < NUM_MOTORS; i++)
                    {
                        Serial.print(motors[i]->name());
                        Serial.print(":\tPosition: "); Serial.print(motors[i]->position(), 4);
                        Serial.print("\tTorque: "); Serial.print(motors[i]->torque(), 4);
                        Serial.print("\tVelocity: "); Serial.println(motors[i]->velocity(), 4);                  
                    }
                    Serial.println();
                }
                digitalWrite(AUX_PIN_1,!digitalRead(AUX_PIN_1));
            }
            current_state = MachineStates::PRINT_MENU;
        }break;

        case SET_POS_ORIGIN:
            Serial.print("\nEstableciendo posicion actual como el origen\n");
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                Serial.print(motors[i]->name());
                if (motors[i]->setCurrentPositionAsOrigin()){
                    Serial.print(":\tPosition: "); Serial.print(motors[i]->position(), 4);
                    Serial.print("\tTorque: "); Serial.print(motors[i]->torque(), 4);
                    Serial.print("\tVelocity: "); Serial.println(motors[i]->velocity(), 4);
                }
                else 
                {
                    Serial.println(":\tFailed setting current position as origin");
                }
            }
            current_state = MachineStates::PRINT_MENU;
            break;

        case SET_POS_ZERO:
        {
            Serial.print("\nSe ha establecido la posicion actual como zero del motor\n");
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                Serial.print(motors[i]->name());
                if (motors[i]->setCurrentPositionAsZero())
                {
                    Serial.print("Position: "); Serial.print(motors[i]->position(), 4);
                    Serial.print("\tTorque: "); Serial.print(motors[i]->torque(), 4);
                    Serial.print("\tVelocity: "); Serial.println(motors[i]->velocity(), 4);
                }
                else 
                {
                    Serial.println(":\tFailed setting current position as origin");
                }
            }
            current_state = PRINT_MENU;
        } break;

        case START_AUTO_MODE:
            Serial.print("Starting auto mode for:"); Serial.println(motors[0]->name());
            motors[0]->startAutoMode([](){motors[0]->handleInterrupt();});
            Serial.print("Starting auto mode for:"); Serial.println(motors[1]->name());
            motors[1]->startAutoMode([](){motors[1]->handleInterrupt();});

            //void(*p[1])() = {motors[0]->handleInterrupt};
            //p[0] = motors[0]->handleInterrupt;

            //auto hi = motors[0]->handleInterrupt;

            current_state = PRINT_MENU;
            break;

        case STOP_AUTO_MODE:
            //Serial.print("Disabling auto mode: "); Serial.println(motor1.name());
            //Serial.print("Disabling auto mode: "); Serial.println(motor2.name());
            //motor1.stopAutoMode();
            //motor2.stopAutoMode();                        
            current_state = PRINT_MENU;
            break;

        default:
            current_state = PRINT_MENU;
            break;
    }

}