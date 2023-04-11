

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

void loop() {
    

    bus.PollCAN(); // Check for messages from the motors.

    long now = millis();
    if (now - last_command >= 10) // Loop at 100Hz. You should limit the rate at which you call CommandTorques to <1kHz to avoid saturating the CAN bus bandwidth
    {
        int max = 32000;
        float left_in = pulseIn(15, HIGH);
        float right_in = pulseIn(14, HIGH);
        if (left_in < 1000 || right_in < 1000){max = 0;} // handle transmitter disconnect

        int left_effort = map(left_in, 1000, 2100, max, -max);
        int right_effort = map(right_in, 1000, 2100, max, -max);

        Serial.println();
        Serial.print("Left 1: ");
        Serial.println(left_in);
        Serial.println(left_effort);
        Serial.print("Right 2: ");
        Serial.println(right_in);
        Serial.println(right_effort);

        int deadzone = 3000;
        
        if(left_effort < deadzone && left_effort > -deadzone){
          left_effort = 0;
        }

        if(right_effort < deadzone && right_effort > -deadzone){
          right_effort = 0;
        }

        // Send Drive Motor Command
        bus.CommandTorques(left_effort, left_effort, right_effort, right_effort, C610Subbus::kOneToFourBlinks);

        float throttle_in = pulseIn(13, HIGH);
        // Serial.println("Throttle 3");
        // Serial.print(throttle_in);


        // read encoder values
        leftEncoderValue = analogRead(leftEncoderPin);
        rightEncoderValue = analogRead(rightEncoderPin);

        // map encoder values to angles
        leftAngle = map(leftEncoderValue, 0, 2047, 0, 360);
        rightAngle = map(rightEncoderValue, 0, 2047, 0, 360);

        // do something with the angles
        // ...
        // Serial.print("Right Angle : ");
        // Serial.println(rightAngle);
        // Serial.print("Left Angle : ");
        // Serial.println(leftAngle);

        // These lines will cause the motors to turn. Make sure they are mounted safely. 
        // bus.CommandTorques(left_effort, left_effort, 0, 0, C610Subbus::kFiveToEightBlinks);      // Command 500mA to motor 5, 600ma to motor 6, etc. The last parameter specifies to command the motors with IDs 5-8.
        float m0_pos = bus.Get(3).Position(); // Get the shaft position of motor 0 in radians.
        float m1_vel = bus.Get(3).Velocity(); // Get the shaft velocity of motor 1 in radians/sec.
        float m2_current = bus.Get(3).Current(); // Get the current estimate of motor 2 in amps.

        last_command = now;
    }

}


