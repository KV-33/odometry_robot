#pragma once

#include <Arduino.h>

class ACTUATOR {
private:
  const int _encA = 0;
  const int _encB = 1;
  int _enc_pins[2] = {0,0};

  const int _motPin1 = 0;
  const int _motPin2 = 1;
  int _mot_pins[2] = {0,0};

  volatile long _enc_counter = 0;
  bool _sign[2];

  void encA(void);
  void encB(void);

public:
  ACTUATOR();
  ~ACTUATOR();
  void init();
  void setMotorPins(int pin1, int pin2);
  void setEncoderPins(int pinA, int pinB);
  unsigned long getEncoderTiks();
  void classInterruptHandler(void);
  
};



