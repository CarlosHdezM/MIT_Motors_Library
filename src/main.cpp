#include "Arduino.h"
#include "MitMotor.h"


#define CS_1 5


MitMotor motor1(MitMotor::AK_10, CS_1, "AK10 1");

void setup(){
    Serial.begin(115200);

    while(!motor1.initialize()){
        Serial.print("Retrying to connect to the MCP2515 of motor "); Serial.println(motor1.name);
        delay(100);
    }
    Serial.println("Initialized succesfully");
}


void loop() {
    if(Serial.available()){
        switch (Serial.read())
        {
        case '1':
            if (motor1.enterMotorMode()){
                Serial.print("Position: "); Serial.println(motor1.position(), 4);
                Serial.print("Torque: "); Serial.println(motor1.torque(), 4);
                Serial.print("Velocity: "); Serial.println(motor1.velocity(), 4);
                Serial.println();
            }
            else Serial.println("No response");
            break;

        case '2':
            if (motor1.setCurrent(0)) Serial.println("Current 0 setpoint sent");
            else Serial.println("Failed sending the message");
            break;

        case '3':
            if (motor1.readMotorResponse()){
                Serial.print("Position: "); Serial.println(motor1.position(), 4);
                Serial.print("Torque: "); Serial.println(motor1.torque(), 4);
                Serial.print("Velocity: "); Serial.println(motor1.velocity(), 4);
                Serial.println();
            }
            else Serial.println("No response");
            break;

        case '4':
            if (motor1.setCurrent(0.7)) Serial.println("Current 0.7 setpoint sent");
            else Serial.println("Failed sending the message");
            break;

        default:
            break;
        }
    }

}