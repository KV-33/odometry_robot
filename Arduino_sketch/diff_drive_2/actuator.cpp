#include "actuator.h"
#include <Arduino.h>

ACTUATOR *pointerToClass; // declare a pointer to testLib class

static void outsideInterruptHandler(void) { // define global handler
  pointerToClass->classInterruptHandler(); // calls class member handler
}

#if defined (PWM_MODE_16)
  #define PWM_MODE PWM
  #define MAX_PWM 65535
  #define MIN_PWM -65535
  #define PWM_write(p,v) pwmWrite(p,constrain(v,MIN_PWM, MAX_PWM))
#else
  #define PWM_MODE OUTPUT
  #define MAX_PWM 255
  #define MIN_PWM -255
  #define PWM_write(p,v) analogWrite(p,constrain(v,MIN_PWM, MAX_PWM))
#endif

ACTUATOR::ACTUATOR()
{
}

ACTUATOR::~ACTUATOR()
{
}

void ACTUATOR::setMotorPins(int pin1, int pin2)
{
  _mot_pins[_motPin1] = pin1;
  _mot_pins[_motPin2] = pin2;
}

void ACTUATOR::setEncoderPins(int pinA, int pinB)
{
  _enc_pins[_encA] = pinA;
  _enc_pins[_encB] = pinB;
}

void ACTUATOR::init()
{
    pinMode(_enc_pins[_encA], INPUT);
    pinMode(_enc_pins[_encB], INPUT);
    _sign[_encA] = digitalRead(_enc_pins[_encA]);
    _sign[_encB] = digitalRead(_enc_pins[_encB]);
    pointerToClass = this;
    
    // attachInterrupt(_enc_pins[_encA], &this.encA, CHANGE);
    // attachInterrupt(_enc_pins[_encB], encB, CHANGE, this);
    pinMode(_mot_pins[_motPin1], PWM_MODE);
    pinMode(_mot_pins[_motPin1], PWM_MODE);
}

void ACTUATOR::encA(void)
{
  _sign[_encA] = digitalRead(_enc_pins[_encA]);
  if (_sign[_encB] == _sign[_encA])
  {
    _enc_counter++;
  }
  else
  {
    _enc_counter--;
  }
}
void ACTUATOR::encB(void)
{
  _sign[_encB] = digitalRead(_enc_pins[_encB]);
  if (_sign[_encB] != _sign[_encA])
  {
    _enc_counter++;
  }
  else
  {
    _enc_counter--;
  }
}

void ACTUATOR::classInterruptHandler(void)
{
  encA();
  encB();
}

unsigned long ACTUATOR::getEncoderTiks()
{
  return _enc_counter;
}