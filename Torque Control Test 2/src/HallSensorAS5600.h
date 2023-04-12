#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <Arduino.h>  // Include Arduino standard library

class HallSensorAS5600 {
  public:
    HallSensorAS5600(uint8_t pin, int offset, boolean doFlip);  // Constructor with pin number as argument
    float read();  // Method to read analog voltage output of hall sensor
  private:
    uint8_t _pin;     // Pin number of hall sensor
    int _offset;      // Angle Offset
    boolean _doFlip;  // Reverse Rotation 
};

#endif