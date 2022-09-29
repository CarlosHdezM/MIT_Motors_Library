#include <SPI.h>
#include "Encoder.h"

#define PWM_0 0
#define PWM_1 1
#define PWM_2 2
#define PWM_3 3
#define PWM_4 4
#define PWM_5 5
#define PWM_6 6
#define PINTOGGLE 10

Encoder * encoders [] = {
  new Encoder(PWM_2)
};

constexpr size_t NUM_ENCODERS = sizeof(encoders) / sizeof(encoders[0]);

void (* interruptsHandlers[NUM_ENCODERS])() = {
  [](){encoders[0]->handlePWM();}
};

void setup()
{
    Serial.begin(115200);
    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        encoders[i]->init();
        encoders[i]->enableInterrupt(interruptsHandlers[i]);
        encoders[i]->setZero();
    }
}

void loop()
{
  for (uint8_t i = 0; i < NUM_ENCODERS; i++)
  {
      Serial.print(encoders[i]->getAngle());
      Serial.print("\t");
  }
  Serial.print("\n");
  delayMicroseconds(100);
}