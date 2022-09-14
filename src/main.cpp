#include "Arduino.h"
#include "machine_state.h"
#include "MitMotor.h"
#include "RmdMotor.h"
#include "array"
#include <algorithm>
 

#define FC 35
#define KD 0.1f
#define KP 1.0f
float P_REF = 0.0;

// Kalman 1
float T = 0.003;
float R11 = 0.001;
float R22 = 0.11;
float Q11 = 0.001;
float Q12 = 0.0001;
float Q21 = 0.0001;
float Q22 = 1;
float P11 = 10;
float P12 = 0;
float P21 = 0;
float P22 = 0.01;
float pjj = 0;
float posk = 0;
float delta = 1;
float velk = 0;
float k11;
float k12;
float k21;
float k22;

float theta = 0;
float vel_stim = 0;
float tau = 0;

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
    //new RmdMotor(RmdMotor::RMD_X6, CS_2, INT_2, "RMD X6 1" )
};
constexpr size_t NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);


void(*interrupt_handlers[NUM_MOTORS])() = {
    [](){motors[0]->handleInterrupt();},
    //[](){motors[1]->handleInterrupt();}
};


IntervalTimer myTimer;

void setup()
{
    Serial.begin(115200);
    pinMode(BOTON, INPUT_PULLUP);
    pinMode(AUX_PIN_1, OUTPUT);
    digitalWrite(AUX_PIN_1,LOW);
    delay(500);

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


void controlMotors()
{
    // Kalman 1
    posk = posk + T * velk;
    velk = velk;
    P11 = P11 + P12 * T + P21 * T + P22 * T * T + Q11;
    P12 = P12 + P22 * T + Q12;
    P21 = P21 + P22 * T + Q21;
    P22 = P22 + Q22;
    delta = (P11 + R11) * (P22 + R22) - P21 * P12;

    k11 = P11 / (P11 + R11);
    k22 = P21 / (P11 + R11);

    velk = velk + k22 * (motors[0]->position() - posk);
    posk = posk + k11 * (motors[0]->position() - posk);

    // velk = velk + k22 * (0 - posk);
    // posk = posk + k11 * (0 - posk);

    P11 = (1 - k11) * P11;
    P12 = (1 - k11) * P12;
    P21 = -k22 * P11 + P21;
    P22 = -k22 * P12 + P22;
    pjj = (P21 + P12) / 2;
    P21 = pjj;
    P12 = pjj;
    tau = -0.5 * ((posk) - 0.0) - 0.3 * velk;
    if (!motors[0]->setTorque(tau,1000))
        Serial.println("Message NOT Sent");
    if(!motors[0]->readMotorResponse(2000)) Serial.println("Message NOT Received");
    // if(!motors[0]->requestPosition()) Serial.println("Message NOT Sent");
    //if (!motors[0]->readMotorResponse(2000))
    //Serial.println("Message NOT Received");
    // Serial.print("Position: ");
    // Serial.print((motors[0]->position()), 4);
    // Serial.print("\tTorque: ");
    // Serial.print(motors[0]->torque(), 4);
    // Serial.print("\tTorque calc: ");
    // Serial.print(tau, 4);
    // Serial.print("\tVelocity: ");
    // Serial.println(motors[0]->velocity(), 4);
    // Serial.println();
    digitalWrite(AUX_PIN_1,!digitalRead(AUX_PIN_1));
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
        myTimer.begin(controlMotors,200);
        myTimer.priority(32);
        theta = -motors[0]->position();
        while (digitalRead(BOTON) == HIGH)
        { 
            //Moved to the interrupt
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
                Serial.print("Starting auto mode for:"); Serial.println(motors[i]->name());
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