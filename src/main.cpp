#include "Arduino.h"
#include "machine_state.h"
#include "MitMotor.h"
#include "RmdMotor.h"
#include "array"
#include <algorithm>

constexpr float PERIOD_USEC = 1000;
constexpr float T = PERIOD_USEC/1000000.0;

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
    new MitMotor(MitMotor::AK_10, CS_1, INT_1, "AK 10 1"),
    new RmdMotor(RmdMotor::RMD_X6, CS_2, INT_2, "RMD X6 1" )
};
constexpr size_t NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);


void(*interrupt_handlers[NUM_MOTORS])() = {
    [](){motors[0]->handleInterrupt();},
    [](){motors[1]->handleInterrupt();}
};

IntervalTimer myTimer;

// Kalman 1
float R11[NUM_MOTORS];
float R22[NUM_MOTORS];
float Q11[NUM_MOTORS];
float Q12[NUM_MOTORS];
float Q21[NUM_MOTORS];
float Q22[NUM_MOTORS];
float P11[NUM_MOTORS];
float P12[NUM_MOTORS];
float P21[NUM_MOTORS];
float P22[NUM_MOTORS];
float pjj[NUM_MOTORS];
float posk[NUM_MOTORS];
float delta[NUM_MOTORS];
float velk[NUM_MOTORS];
float k11[NUM_MOTORS];
float k22[NUM_MOTORS];
float tau[NUM_MOTORS];


void setup()
{
    Serial.begin(115200);
    pinMode(BOTON, INPUT_PULLUP);
    pinMode(AUX_PIN_1, OUTPUT);
    digitalWrite(AUX_PIN_1,LOW);
    delay(500);


    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        R11[i] = 0.001;
        R22[i] = 0.11;
        Q11[i] = 0.001;
        Q12[i] = 0.0001;
        Q21[i] = 0.0001;
        Q22[i] = 1;
        P11[i] = 10;
        P12[i] = 0;
        P21[i] = 0;
        P22[i] = 0.01;
        posk[i] = 0;
        delta[i] = 1;
        velk[i] = 0;
        tau[i] = 0;
    }   

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
    Serial.println("All motors initialized succesfullyy");

    Serial.print("Tau: "); Serial.println(T,10);
}


void controlMotors()
{
    digitalWrite(AUX_PIN_1,!digitalRead(AUX_PIN_1));
    for(uint8_t i=0; i < NUM_MOTORS; i++)
    {
        // Kalmans
        posk[i] = posk[i] + T * velk[i];
        velk[i] = velk[i];
        P11[i] = P11[i] + P12[i] * T + P21[i] * T + P22[i] * T * T + Q11[i];
        P12[i] = P12[i] + P22[i] * T + Q12[i];
        P21[i] = P21[i] + P22[i] * T + Q21[i];
        P22[i] = P22[i] + Q22[i];
        delta[i] = (P11[i] + R11[i]) * (P22[i] + R22[i]) - P21[i] * P12[i];

        k11[i] = P11[i] / (P11[i] + R11[i]);
        k22[i] = P21[i] / (P11[i] + R11[i]);

        velk[i] = velk[i] + k22[i] * (motors[i]->position() - posk[i]);
        posk[i] = posk[i] + k11[i] * (motors[i]->position() - posk[i]);

        P11[i] = (1 - k11[i]) * P11[i];
        P12[i] = (1 - k11[i]) * P12[i];
        P21[i] = -k22[i] * P11[i] + P21[i];
        P22[i] = -k22[i] * P12[i] + P22[i];
        pjj[i] = (P21[i] + P12[i]) / 2;
        P21[i] = pjj[i];
        P12[i] = pjj[i];
        tau[i] = -0.5 * ((posk[i]) - 0.0) - 0.1 * velk[i];
        if (!motors[i]->setTorque(tau[i]))
        {
            Serial.print("Message NOT Sent to "); Serial.println(motors[i]->name());
        }
        //if(!motors[i]->readMotorResponse(2000)) Serial.println("Message NOT Received");
    }
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
        // for (uint8_t i = 0; i < NUM_MOTORS; i++)
        // {
        //     Serial.print("Starting auto mode for:"); Serial.println(motors[i]->name());
        //     motors[i]->startAutoMode(interrupt_handlers[i]);
        // }
        Serial.println("\nVamos a escribir y leer continuamente");
        myTimer.begin(controlMotors,PERIOD_USEC);
        myTimer.priority(32);
        while (digitalRead(BOTON) == HIGH)
        { 
            //Moved to the interrupt
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                Serial.print("Motor "); Serial.print(motors[i]->name()); 
                Serial.print(":\tPosition: "); Serial.print(motors[i]->position(), 4);
                Serial.print("\tTorque: "); Serial.print(motors[i]->torque(), 4);
                Serial.print("\tVelocity: "); Serial.println(motors[i]->velocity(), 4);
                Serial.println();
            }
            delay(20);
        }
        myTimer.end();
        for (uint8_t i = 0; i < NUM_MOTORS; i++)
        {
            Serial.print("Disabling auto mode for:"); Serial.println(motors[i]->name());
            motors[i]->stopAutoMode();
            while (!motors[i]->setTorque(0, 1000)){}
            while (!motors[i]->readMotorResponse(2000)){}
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
        {
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                Serial.print("Starting auto modee for:"); Serial.println(motors[i]->name());
                motors[i]->startAutoMode(interrupt_handlers[i]);
            }
            current_state = PRINT_MENU;
        }break;


        case STOP_AUTO_MODE:
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                Serial.print("Stopping auto mode for:"); Serial.println(motors[i]->name());
                motors[i]->stopAutoMode();
            }                      
            current_state = PRINT_MENU;
            break;


        default:
            current_state = PRINT_MENU;
            break;
    }

}