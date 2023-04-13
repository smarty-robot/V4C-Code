#include <Arduino.h>
#include "C610Bus.h"
#include <iostream>
#include <string.h>
#include <stdio.h>
#include "pid.h"
#include "HallSensorAS5600.h"  // Include HallSensor library

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

void setup() {
  pinMode(15, INPUT); // left
  pinMode(14, INPUT); // right
  pinMode(13, INPUT); // wings
}

HallSensorAS5600 hallLeft(A6, -55, false);  // Create HallSensor object with pin number as argument
HallSensorAS5600 hallRight(A5, -115, false);  // Create HallSensor object with pin number as argument

double drive_p = 100; // position
double drive_i = 10; // integral
double drive_d = 5; // derivative
double drive_dt = .1; //ms
double currentLimit = 20000;
PID pidBL = PID(drive_dt, currentLimit, -currentLimit, 80, drive_d, drive_i);
PID pidFL = PID(drive_dt, currentLimit, -currentLimit, drive_p, drive_d, drive_i);
PID pidBR = PID(drive_dt, currentLimit, -currentLimit, drive_p, drive_d, drive_i);
PID pidFR = PID(drive_dt, currentLimit, -currentLimit, drive_p, drive_d, drive_i);

PID pidWL = PID(.06, 1000, -1000, 100, 10, 0);
PID pidWR = PID(.06, 1000, -1000, 100, 10, 0);


int iterator = 0;
int hold = 0;
float maxRadPerSec = 50.0;

boolean doVelocityControl = false;
boolean doEffortControl = true;
boolean doWings = true;

void loop() {

    bus.PollCAN(); // Check for messages from the motors.
    //bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
    iterator++;

    long now = millis();
    if (now - last_command >= 10) // Loop at 100Hz. You should limit the rate at which you call CommandTorques to <1kHz to avoid saturating the CAN bus bandwidth
    {
        //Serial.println(now - last_command);
        last_command = millis();

        // Drive 
        // Velocity Control
        if(doVelocityControl){
          float leftIn = pulseIn(15, HIGH);
          int leftPer = map(leftIn, 900, 2100, -100, 100);
          float leftVelTarget = map(leftPer, -100, 100, -maxRadPerSec, maxRadPerSec);

          float rightIn = pulseIn(14, HIGH);
          int rightPer = map(rightIn, 900, 2100, -100, 100);
          float rightVelTarget = map(rightPer, -100, 100, -maxRadPerSec, maxRadPerSec);

          double actualVel[4];
          for (int i = 0; i < 4; i++){
            actualVel[i] = bus.Get(i).Velocity();
          }
          int torqueMtr[4] = {0, 0, 0, 0};
          torqueMtr[0] = pidBL.calculate(leftVelTarget, actualVel[0]);
          torqueMtr[1] = pidFL.calculate(leftVelTarget, actualVel[1]);
          torqueMtr[2] = pidBR.calculate(rightVelTarget, actualVel[2]);
          torqueMtr[3] = pidFR.calculate(rightVelTarget, actualVel[3]);

          bus.CommandTorques(torqueMtr[0], torqueMtr[1], torqueMtr[2], torqueMtr[3], C610Subbus::kOneToFourBlinks);

          if (false && iterator%5 == 0){//print maps
          Serial.println();

          Serial.print("Time Past:      ");
          Serial.println((millis() - hold));
          hold = millis();

          Serial.print("Left Duty:      ");
          Serial.println(leftIn);
          Serial.print("Left Per:       ");
          Serial.println(leftPer);
          Serial.print("Left Velocity:  ");
          Serial.println(leftVelTarget);

          Serial.print("Right Duty:     ");
          Serial.println(rightIn);
          Serial.print("Right Per:      ");
          Serial.println(rightPer);
          Serial.print("Right Velocity: ");
          Serial.println(rightVelTarget);

          Serial.print("Actual Vel      ");
          for (int i = 0; i < 4; i++){
            Serial.print(actualVel[i]);
            Serial.print("  ");
          }
          Serial.println();

          Serial.print("System Input    ");
          for (int i = 0; i < 4; i++){
            Serial.print(torqueMtr[i]);
            Serial.print("  ");
          }
          Serial.println();
        }
        }
        // Effort Control
        if(doEffortControl){
          float leftIn = pulseIn(15, HIGH);
          float rightIn = pulseIn(14, HIGH);

          int max_amperage = 16000;
          int leftPer = map(leftIn, 900, 2100, -max_amperage, max_amperage);
          int rightPer = map(rightIn, 900, 2100, -max_amperage, max_amperage);
          
          if(leftPer > -500 && leftPer < 500){
            leftPer = 0;
          }
          if(rightPer > -500 && rightPer < 500){
            rightPer = 0;
          }

          bus.CommandTorques(-leftPer, -leftPer, -rightPer, -rightPer, C610Subbus::kOneToFourBlinks);

        }
        
        // Wing
        // Wing Positioning      
        if (doWings){
          float wingIn = pulseIn(13, HIGH);
          float wingPosTarget = map(wingIn, 1100, 1900, 0, 90);

          float leftWingPos = hallLeft.read();
          int leftWingTorq = pidWL.calculate(wingPosTarget, leftWingPos);

          float rightWingPos = hallRight.read();
          int rightWingTorq = pidWR.calculate(wingPosTarget, rightWingPos);

          bus.CommandTorques(leftWingTorq, rightWingTorq,0,0, C610Subbus::kFiveToEightBlinks);

          if(iterator%50 == 0){
            Serial.println();
            Serial.print("wing target positon:   ");
            Serial.println(wingPosTarget);
            Serial.print("Left Wing Position:    ");
            Serial.println(leftWingPos);
            Serial.print("Left Wing Torque:      ");
            Serial.println(leftWingTorq);
            Serial.print("Right Wing Position:   ");
            Serial.println(rightWingPos);
            Serial.print("Right Wing Torque:     ");
            Serial.println(rightWingTorq);
          }
        }
        
    }
}