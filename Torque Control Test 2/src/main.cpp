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
  bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
  bus.CommandTorques(0, 0, 0, 0, C610Subbus::kFiveToEightBlinks);
}

HallSensorAS5600 hallLeft(A6, -51, false);  // Create HallSensor object with pin number as argument
HallSensorAS5600 hallRight(A5, -114, false);  // Create HallSensor object with pin number as argument


double drive_p = 100; // position
double drive_i = 10; // integral
double drive_d = 5; // derivative
double drive_dt = .1; //ms
double driveCurrLimit = 20000;
PID pidBL = PID(drive_dt, driveCurrLimit, -driveCurrLimit, 80, drive_d, drive_i);
PID pidFL = PID(drive_dt, driveCurrLimit, -driveCurrLimit, drive_p, drive_d, drive_i);
PID pidBR = PID(drive_dt, driveCurrLimit, -driveCurrLimit, drive_p, drive_d, drive_i);
PID pidFR = PID(drive_dt, driveCurrLimit, -driveCurrLimit, drive_p, drive_d, drive_i);

float wingPosTarget;
float wingIn;
float leftWingPos;
int leftWingTorq;
float rightWingPos;
int rightWingTorq;

int wing_p = 400;
int wing_d = 50;
int wing_i = 0;

double wingCurrLimit = 5000;
PID pidWL = PID(.06, wingCurrLimit, -wingCurrLimit, wing_p, wing_d, wing_i);
PID pidWR = PID(.06, wingCurrLimit, -wingCurrLimit, wing_p, wing_d, wing_i);


int iterator = 0;
int hold = 0;
float maxRadPerSec = 50.0;

boolean doMotorOut;
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
        float leftIn = pulseIn(15, HIGH);
        float leftPer = map(leftIn, 900, 2100, -100, 100);
        float rightIn = pulseIn(14, HIGH);
        float rightPer = map(rightIn, 900, 2100, -100, 100);

        
        if (leftIn > 500 && rightIn > 500){
          // Velocity Control
          if(doVelocityControl){
            
            float leftVelTarget = map(leftPer, -100, 100, -maxRadPerSec, maxRadPerSec);
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

            if (true && iterator%5 == 0){//print maps
              Serial.println();

              Serial.print("Time Past:      ");
              Serial.println((millis() - hold));
              hold = millis();

              Serial.print("Left Velocity:  ");
              Serial.println(leftVelTarget);

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
            int leftTorq = map(leftPer, -100, 100, -driveCurrLimit, driveCurrLimit);
            int rightTorq = map(rightPer, -100, 100, -driveCurrLimit, driveCurrLimit);
        
            if(leftTorq > -500 && leftTorq < 500){
              leftTorq = 0;
            }
            if(rightTorq > -500 && rightTorq < 500){
              rightTorq = 0;
            }

            bus.CommandTorques(-leftTorq, -leftTorq, -rightTorq, -rightTorq, C610Subbus::kOneToFourBlinks);

            if (false && iterator%50 == 0){
              Serial.print("Left Duty:      ");
              Serial.println(leftIn);
              Serial.print("Left Per:       ");
              Serial.println(leftPer);
              Serial.print("leftTorq:       ");
              Serial.println(leftTorq);

              Serial.print("Right Duty:     ");
              Serial.println(rightIn);
              Serial.print("Right Per:      ");
              Serial.println(rightPer);
              Serial.print("leftTorq:       ");
              Serial.println(rightTorq);
            }
          }
        }

        

        // Wing
        // Wing Positioning      
        if (doWings){
          wingIn = pulseIn(13, HIGH);
          wingPosTarget = map(wingIn, 1100, 1900, 0, 90);

          leftWingPos = hallLeft.read();
          leftWingTorq = pidWL.calculate(wingPosTarget, leftWingPos);

          rightWingPos = hallRight.read();
          rightWingTorq = pidWR.calculate(wingPosTarget, rightWingPos);

          bus.CommandTorques(leftWingTorq, rightWingTorq, 0, 0, C610Subbus::kFiveToEightBlinks);

          if(true && iterator%50 == 0){
            Serial.println();
            Serial.print("wing in:               ");
            Serial.println(wingIn);
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