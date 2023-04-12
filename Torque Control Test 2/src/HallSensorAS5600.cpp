#include "HallSensorAS5600.h"
#include <math.h>

HallSensorAS5600::HallSensorAS5600(uint8_t pin, int offset = 0, boolean doFlip = false) {
  _pin = pin;
  _offset = offset;
  _doFlip = doFlip;
}

float HallSensorAS5600::read() {
  float angle = analogRead(_pin) * (360 / 1023.0);
  if (_doFlip){angle = -angle;}
  angle += _offset;
  angle = fmod(angle, 360);
  return angle;  // Convert ADC reading to voltage (assuming 5V reference)
}
