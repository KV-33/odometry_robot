#include "actuator.h"

#define PWM_MODE_16
#define MOTOR_COUNT 2
#define LEFT 0
#define RIGHT 1

ACTUATOR actuators[MOTOR_COUNT];

void setup()
{
    Serial.begin(9600);
    actuators[LEFT].setEncoderPins(PA9, PA10);
    actuators[LEFT].init();
}

void loop()
{
    Serial.println(actuators[LEFT].getEncoderTiks());
}
