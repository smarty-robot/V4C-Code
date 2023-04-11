

#include <Arduino.h>
#include "C610Bus.h"
 
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
  pinMode(14, INPUT); // right
  pinMode(13, INPUT); // wings
}

int iterator = 10;

void loop() {
    
    bus.PollCAN(); // Check for messages from the motors.
    //bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);

    long now = millis();
    if (now - last_command >= 10) // Loop at 100Hz. You should limit the rate at which you call CommandTorques to <1kHz to avoid saturating the CAN bus bandwidth
    {

        //delay(1000);

        float leftIn = pulseIn(15, HIGH);      // Left Velocity   1006 - 2026
        float rightIn = pulseIn(14, HIGH);     // Right Velocity  1005 - 2027
        float armPosIn = pulseIn(13, HIGH);    // Arm Position    1100 - 1900

        if (false){//print maps
          Serial.print("Left 1: ");
          Serial.println(leftIn);
          Serial.print("Right 2: ");
          Serial.println(rightIn);
          Serial.print("Arm Pos: ");
          Serial.println(armPosIn); 
        } 

        int max = 100;
        int leftPer = map(leftIn, 900, 2100, max, -max);
        int rightPer = map(rightIn, 900, 2100, max, -max);
        int armPer = map(armPosIn, 1100, 1900, -90, 0);

        if (false){//print maps
          Serial.print("Left 1: ");
          Serial.println(leftPer);
          Serial.print("Right 2: ");
          Serial.println(rightPer);
          Serial.print("Arm Pos: ");
          Serial.println(armPer); 
        } 

        float maxRadPerSec = 50.0;

        float leftVelTarget = map(leftPer, -100, 100, -maxRadPerSec, maxRadPerSec);
        float rightVelTarget = map(rightPer, -100, 100, -maxRadPerSec, maxRadPerSec);

        double motorVels[4] = {0,1,2,3};
        for (int i = 0; i < 4; i++){
          motorVels[i] = bus.Get(i).Velocity();
        }

        if(iterator%40 == 0){//print Vel Target
          bus.CommandTorques(500, 500, -500, -500, C610Subbus::kOneToFourBlinks);
          Serial.print("Left Velocity Target: ");
          Serial.println(leftVelTarget);
          Serial.print("Right Velocity Target: ");
          Serial.println(rightVelTarget);
          Serial.print("Motor Velocities");

        // if (false || iterator%40 == 20){
        //   Serial.println();
        //   int ct = 300;
        //   Serial.println("Current Commanded");
        //   Serial.println(ct);
        //   bus.CommandTorques(ct, 0, 0, 0, C610Subbus::kOneToFourBlinks);
        //   float m2_current = bus.Get(0).Current(); // Get the current estimate of motor 2 in amps.
        //   Serial.println("Current");
        //   Serial.println(m2_current);
        //   Serial.println("Velocity");
        //   float m1_vel_backLeft = bus.Get(0).Velocity();
        //   Serial.println(m1_vel_backLeft);
        // }
        iterator++;
    }
    }
}


