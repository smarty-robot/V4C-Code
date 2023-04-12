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

HallSensorAS5600 hallLeft(A6, 261, true);  // Create HallSensor object with pin number as argument
HallSensorAS5600 hallRight(A5, -218, false);  // Create HallSensor object with pin number as argument

double drive_p = 100; // position
double drive_i = 10; // integral
double drive_d = 5; // derivative
double drive_dt = .1; //ms
double currentLimit = 20000;
PID pidBL = PID(drive_dt, currentLimit, -currentLimit, 80, drive_d, drive_i);
PID pidFL = PID(drive_dt, currentLimit, -currentLimit, drive_p, drive_d, drive_i);
PID pidBR = PID(drive_dt, currentLimit, -currentLimit, drive_p, drive_d, drive_i);
PID pidFR = PID(drive_dt, currentLimit, -currentLimit, drive_p, drive_d, drive_i);

PID pidWL = PID(.1, 1000, -1000, 100, 10, 30);


int iterator = 0;
int hold = 0;
float maxRadPerSec = 50.0;

void loop() {

    bus.PollCAN(); // Check for messages from the motors.
    //bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);

    long now = millis();
    if (now - last_command >= 10) // Loop at 100Hz. You should limit the rate at which you call CommandTorques to <1kHz to avoid saturating the CAN bus bandwidth
    {
        //Serial.println(now - last_command);
        last_command = millis();
        int max = 100;

        float leftIn = pulseIn(15, HIGH);
        int leftPer = map(leftIn, 900, 2100, -max, max);
        float leftVelTarget = map(leftPer, -100, 100, -maxRadPerSec, maxRadPerSec);

        float rightIn = pulseIn(14, HIGH);
        int rightPer = map(rightIn, 900, 2100, -max, max);
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

        // bus.CommandTorques(250,250,-250,-250, C610Subbus::kOneToFourBlinks);
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
        iterator++;

        float wingIn = pulseIn(13, HIGH);
        float wingPosTarget = map(wingIn, 1100, 1900, 0, 180);
        float wingPos = hallLeft.read();
        int leftWingTorq = pidWL.calculate(wingPosTarget, wingPos);
        bus.CommandTorques(-leftWingTorq, 0,0,0, C610Subbus::kFiveToEightBlinks);

        if(iterator%50 == 0){
          Serial.print("wing target positon:    ");
          Serial.println(wingPosTarget);
          Serial.print("Left Wing Position:     ");
          Serial.println(wingPos);
          Serial.print("Left Wing Torque:       ");
          Serial.println(leftWingTorq);
        }
    }
}