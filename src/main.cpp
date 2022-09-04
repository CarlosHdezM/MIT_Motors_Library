#include "Arduino.h"
#include "machine_state.h"
#include "MitMotor.h"
#include "RmdMotor.h"
 
#define CS_1 2
#define INT_1 27
#define CS_2 3
#define INT_2 28


#define BOTON 1
#define AUX_PIN_1 9


#define MOTOR_RMD 0
#if MOTOR_RMD
    RmdMotor motor1(RmdMotor::RMD_L5015, CS_1, "RMD_L5015");

#else
    MitMotor motor1(MitMotor::AK_10, CS_1, INT_1, "AK_10 1");
    //MitMotor motor2(MitMotor::GIM, CS_2, INT_2, "GIM 1");
#endif

RmdMotor motor2(RmdMotor::RMD_X6, CS_2, INT_2, "RMD X6 1" );

MachineStates current_state = MachineStates::PRINT_MENU;


void setup()
{
    Serial.begin(115200);

    pinMode(BOTON, INPUT_PULLUP);
    pinMode(AUX_PIN_1, OUTPUT);
    digitalWrite(AUX_PIN_1,LOW);

    while(!motor1.initialize()){
        Serial.print("Retrying to connect to the MCP2515 of motor "); Serial.println(motor1.name());
        delay(100);
    }

    while(!motor2.initialize()){
        Serial.print("Retrying to connect to the MCP2515 of motor "); Serial.println(motor1.name());
        delay(100);
    }


    Serial.println("Initializedd succesfully");


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
            if (motor1.turnOn())
            {
                Serial.print("Position: "); Serial.print(motor1.position(), 4);
                Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);
                Serial.println();
            }
            else Serial.println("No response");
            current_state = MachineStates::PRINT_MENU;
            break;

        case DISABLE_MOTORS:
            Serial.println("\nDisabling Motor");   
            if (motor1.turnOff())
            {
                Serial.print("Position: "); Serial.print(motor1.position(), 4);
                Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);
                Serial.println();
            }
            else Serial.println("No response");  
            current_state = MachineStates::PRINT_MENU;
            break;

        case SET_TORQUE_ZERO:
            Serial.print("\nEstableciendo el torque a 0\n");
            if (motor1.setTorque(0)) Serial.println("Current 0 setpoint sent to motor 1");
            else Serial.println("Failed sending the message to motor 1");
            if (motor2.setTorque(0)) Serial.println("Current 0 setpoint sent to motor 2");
            else Serial.println("Failed sending the message to motor 2");
            current_state = MachineStates::PRINT_MENU;
            break;

        case READ_MOTOR_RESPONSE:
            motor1.readMotorResponse();
            motor2.readMotorResponse();
            Serial.print("\nL Respuesta motores\n");
            Serial.print("Position: "); Serial.print(motor1.position(), 4);
            Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
            Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);

            Serial.print("Position 2: "); Serial.print(motor2.position(), 4);
            Serial.print("\tTorque 2: "); Serial.print(motor2.torque(), 4);
            Serial.print("\tVelocity 2: "); Serial.println(motor2.velocity(), 4);

            current_state = MachineStates::PRINT_MENU;
            break;

        case REQUEST_POSITION:
            Serial.print("\nSolicitando la posicion actual del motor\n");
            #if MOTOR_RMD
                if (motor1.requestPosition()) Serial.println("Sent CAN message to request position");
                else Serial.println("Failed sending the message");
            #endif
            current_state = MachineStates::PRINT_MENU;
            break;

        case SET_TORQUE:
            Serial.print("\nSe ha establecido un valor torque (0.3)\n");
            if (motor1.setTorque(0.3)) Serial.println("Current 0.3 setpoint sent");
            else Serial.println("Failed sending the message");
            current_state = MachineStates::PRINT_MENU;
            break;

        case SET_TORQUE_AND_READ:
        {
            Serial.println("\nVamos a escribir y leer continuamente");
            elapsedMicros wait;
            const uint16_t delay_us = 250;
            while (digitalRead(BOTON) == HIGH)
            {

                //while(wait < delay_us){}
                //wait = 0;

                motor2.setTorque(0,2000);
                motor1.setTorque(0,2000);
                motor2.readMotorResponse(1000);
                motor2.requestPosition();
                motor1.readMotorResponse(2000);
                motor2.readMotorResponse(2000);

                // Serial.print("Position: "); Serial.print(motor1.position(), 4);
                // Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                // Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);

                // Serial.print("Position 2: "); Serial.print(motor2.position(), 4);
                // Serial.print("\tTorque 2: "); Serial.print(motor2.torque(), 4);
                // Serial.print("\tVelocity 2: "); Serial.println(motor2.velocity(), 4);                   
                // Serial.println();
                
                digitalWrite(AUX_PIN_1,!digitalRead(AUX_PIN_1));
            }
            current_state = MachineStates::PRINT_MENU;
        }break;

        case SET_POS_ORIGIN:
            Serial.print("\nSe ha establecido la posicion actual como el origen\n");
            if (motor1.setCurrentPositionAsOrigin()){
                Serial.print("Position: "); Serial.print(motor1.position(), 4);
                Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);
            }
            else 
            {
                Serial.println("Failed setting current position as origin");
                }
            current_state = MachineStates::PRINT_MENU;
            break;

        case SET_POS_ZERO:
        {
            Serial.print("\nSe ha establecido la posicion actual como zero del motor\n");
            if (motor1.setCurrentPositionAsZero())
            {
                Serial.print("Position: "); Serial.print(motor1.position(), 4);
                Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);
                Serial.println();
            }
            else Serial.println("No response");
            current_state = PRINT_MENU;
        } break;

        case START_AUTO_MODE:
            motor1.startAutoMode([](){motor1.handleInterrupt();});
            motor2.startAutoMode([](){motor2.handleInterrupt();});                        
            current_state = PRINT_MENU;
            break;

        case STOP_AUTO_MODE:
            motor1.stopAutoMode();
            motor2.stopAutoMode();                        
            current_state = PRINT_MENU;
            break;

        default:
            current_state = PRINT_MENU;
            break;
    }

}