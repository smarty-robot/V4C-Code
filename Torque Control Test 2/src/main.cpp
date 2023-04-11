#include <Arduino.h>
#include "C610Bus.h"
#include <iostream>
#include <string.h>
#include <stdio.h>
#include "pid.h"
 
long last_command = 0;
C610Bus<CAN1> bus; // Initialization. Templated to either use CAN1 or CAN2.

/*
CAN ID Bus Chart
1 - Back Left
2 - Front Left
3 - Back Right
4 - Front Right
5 - Shoulder Left
6 - Shoulder Right
*/

const int leftEncoderPin = A6;
const int rightEncoderPin = A5;

int leftEncoderValue, rightEncoderValue;
float leftAngle, rightAngle;
float flying[2] = {110,90};
float driving[2] = {180, 160};

void setup() {
  // initialize encoder pins as inputs
  pinMode(leftEncoderPin, INPUT);
  pinMode(rightEncoderPin, INPUT);

  pinMode(15, INPUT); // left
  // pinMode(14, INPUT); // right
  // pinMode(13, INPUT); // wings
}


double p = 100; // position
double i = 10; // integral
double d = 0; // derivative
double dt = 0.05; //ms
double currentLimit = 3200;
PID motor_BL_PID = PID(dt, currentLimit, -currentLimit, p, d, i);

int iterator = 10;
int then = 0;
int itsbeen = 0;

void loop() {

    bus.PollCAN(); // Check for messages from the motors.
    //bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);

    long now = millis();
    if (now - last_command >= 10) // Loop at 100Hz. You should limit the rate at which you call CommandTorques to <1kHz to avoid saturating the CAN bus bandwidth
    {

        //delay(1000);

        float leftIn = pulseIn(15, HIGH);      // Left Velocity   1006 - 2026
        
        int max = 100;
        int leftPer = map(leftIn, 900, 2100, max, -max);
        
        float maxRadPerSec = 50.0;

        float leftVelTarget = map(leftPer, -100, 100, -maxRadPerSec, maxRadPerSec);

        double actualVel = bus.Get(0).Velocity();

        int torqBL = motor_BL_PID.calculate(leftVelTarget, actualVel);
        
        bus.CommandTorques(torqBL, 0, 0, 0, C610Subbus::kOneToFourBlinks);

        if (iterator%40 == 0){//print maps
          Serial.println();

          Serial.print("Left Duty:    ");
          Serial.println(leftIn);
        
          Serial.print("Left Per:     ");
          Serial.println(leftPer);

          Serial.print("Target Vel    ");
          Serial.println(leftVelTarget);

          Serial.print("Actual Vel    ");
          Serial.println(actualVel);

          Serial.print("System Input  ");
          Serial.println(torqBL);
        }
      
        // if(iterator%40 == 0){//print Vel Target
        //   Serial.println();
        //   Serial.print("Left Velocity Target: ");
        //   Serial.println(leftVelTarget);
        //   Serial.println("Motor Velocities");
        //   for (int i = 0; i < 1; i++){
        //     Serial.print("    M");
        //     Serial.print(i);
        //     Serial.print("  ");
        //     Serial.print(motorVels[i]);
        //     Serial.println(" rps");
        //   }
        // }

        iterator++;
        
    }
}


