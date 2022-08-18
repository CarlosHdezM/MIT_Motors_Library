#include "Arduino.h"
#include "MitMotor.h"


#define CS_1 5

//MitMotor motor0(CS_1);
//MitMotor motor1(MitMotor::GIM, CS_1);


void setup(){
    Serial.begin(115200);
    //motor0.initialize();
}


void loop() {
    Serial.print(MitMotor::AK_10.T_MIN);
    Serial.print('\t');
    Serial.println(MitMotor::AK_10.T_MAX);

    Serial.print(MitMotor::GIM.T_MIN);
    Serial.print('\t');
    Serial.println(MitMotor::GIM.T_MAX);
    
    //Serial.print("My max is:" ); Serial.println(motor1.m_motor_type.T_MAX);

    Serial.println();
}