#include "Arduino.h"

#include "MitMotor.h"
#include "RmdMotor.h"
 

#define CS_1 5
#define PIN_BOTON 26

#define MOTOR_RMD 0

#if MOTOR_RMD
    RmdMotor motor1(RmdMotor::RMD_X6, CS_1, "RMD_X6");

#else
    MitMotor motor1(MitMotor::GIM, CS_1, "GIM 1");

#endif


void setup(){
    Serial.begin(115200);

    pinMode(PIN_BOTON, INPUT_PULLUP);   

    while(!motor1.initialize()){
        Serial.print("Retrying to connect to the MCP2515 of motor "); Serial.println(motor1.name());
        delay(100);
    }
    Serial.println("Initialized succesfully");
}


void loop() {
    if(Serial.available()){
        switch (Serial.read())
        {
        case '0':
            if (motor1.turnOff()){
                Serial.print("Position: "); Serial.print(motor1.position(), 4);
                Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);
                Serial.println();
            }
            else Serial.println("No response");
            break;

        case '1':
            if (motor1.turnOn()){
                Serial.print("Position: "); Serial.print(motor1.position(), 4);
                Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);
            }
            else Serial.println("No response");
            break;

        case '2':
            if (motor1.setTorque(0)) Serial.println("Current 0 setpoint sent");
            else Serial.println("Failed sending the message");
            break;

        case '3':
            if (motor1.readMotorResponse()){
                Serial.print("Position: "); Serial.print(motor1.position(), 4);
                Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);
            }
            else Serial.println("No response");
            break;

        case '4':
            #if MOTOR_RMD
                if (motor1.requestPosition()) Serial.println("Sent CAN message to request position");
                else Serial.println("Failed sending the message");
            #endif
            break;

        case '5':
            if (motor1.setTorque(50)) Serial.println("Current 50 setpoint sent");
            else Serial.println("Failed sending the message");
            break;

        case '6':
            while(digitalRead(PIN_BOTON)){
                if(!motor1.setTorque(0,1000)) Serial.println("Message NOT Sent");
                if(motor1.readMotorResponse(2000)){
                    Serial.print("Position: "); Serial.print(motor1.position(), 4);
                    Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                    Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);
                    Serial.println();
                }
                else {
                    Serial.println("No response received!!!");
                }
            }
            break;

        case '7':
            if (motor1.setCurrentPositionAsOrigin()){
                Serial.print("Position: "); Serial.print(motor1.position(), 4);
                Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);
            }
            else {Serial.println("Failed setting current position as origin. Quecuriosojorge");}
            break;

        case 'Z':
            if (motor1.setCurrentPositionAsZero()){
                Serial.print("Position: "); Serial.print(motor1.position(), 4);
                Serial.print("\tTorque: "); Serial.print(motor1.torque(), 4);
                Serial.print("\tVelocity: "); Serial.println(motor1.velocity(), 4);
                Serial.println();
            }
            else Serial.println("No response");
            break;

        default:
            break;
        }
    }

}