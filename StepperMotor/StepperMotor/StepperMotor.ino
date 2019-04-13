#include <Servo.h>
#include <ros.h>
#include "Mobility.h"
#include <AccelStepper.h>

/**
 * DESCRIPTION: This sketch is responsible for controlling the motors that steer the robot. Queries 4 
 * times a second. When the digital read pin reads low, it is not in the range of the magnetic center. 
 * The pin assignments are as follows:
 *  D2: FL DIR    (Pin 2)
 *  D3: FL PUL    (Pin 3)
 *  D4: FL INDEX  (Pin 4)
 *  D5: FR DIR    (Pin 5)
 *  D6: FR PUL    (Pin 6)
 *  D7: FR INDEX  (Pin 7)
 *  D8: BL DIR    (Pin 8)
 *  D9: BL PUL    (Pin 9)
 *  D10: BL INDEX (Pin 10)
 *  D11: BR DIR   (Pin 11)
 *  D12: BR PUL   (Pin 12)
 *  A1: BR INDEX  (Pin A1)
 *  A2: EN        (Pin A2)
 */

void messageCb(const motion_control::Mobility & toggle_msg);

ros::NodeHandle nh;
ros::Subscriber<motion_control::Mobility> sub("steering", &messageCb );

AccelStepper FL(AccelStepper::DRIVER, 3, 2);
AccelStepper FR(AccelStepper::DRIVER, 6, 5);
AccelStepper RL(AccelStepper::DRIVER, 9, 8);
AccelStepper RR(AccelStepper::DRIVER, 12, 11);

int numSteps;
int center;
int QUARTER_ROTATE = 220;
boolean wheelCounterClockwise = true;
boolean wheelClockwise = false;

/**
 * This is the method that handles the ROS message. It will determine the location that
 * it wants each of its wheels to move to.
 */
void messageCb(const motion_control::Mobility & toggle_msg)
{
  FL.moveTo(toggle_msg.front_left);
  FR.moveTo(toggle_msg.front_right);
  RL.moveTo(toggle_msg.rear_left);
  RR.moveTo(toggle_msg.rear_right);
}

/*
 * This method calibrates all the motors to determine their centers. The wheels will only rotate
 * a quarter of a full rotation to prevent damage. This method requires that the wheels be within
 * 90 degrees of their aligned position.
 */
void calibrateMotor(AccelStepper step, int motorIndex)
{
  // First check if the motor is already in the low range; if so, run
  // A different calibration algorithm
  if (digitalRead(motorIndex) == LOW)
  {
    // First run the motor counter clockwise to the edge of the low read
    step.moveTo(QUARTER_ROTATE);
    while (digitalRead(motorIndex) == LOW)
    {
      delayMicroseconds(10);
      step.run();
    }

    // Then run the counter clockwise wheel calibration method
    step.setCurrentPosition(0);
    step.moveTo(-1 * QUARTER_ROTATE);
   while (digitalRead(motorIndex) == LOW)
   {
      delayMicroseconds(10);
      step.run();
   }
    numSteps = step.currentPosition();
    center = numSteps / 2;

    step.setCurrentPosition(0);
    step.moveTo(-1 * center);
    while (step.run()) 
    {
      delayMicroseconds(10);
    }
    step.setCurrentPosition(0);
    return;
  }
  
  // Run the motor counter clockwise first to see if it can find the center
  step.moveTo(QUARTER_ROTATE);
  while (digitalRead(motorIndex) != LOW)
  {
    delayMicroseconds(10);
    if (!step.run())
    { 
      wheelCounterClockwise = false;
      wheelClockwise = true;
      delayMicroseconds(10);
      break;
    }
  }
  
  // If the motor did not reach the full 45 degree turn, run the motor 90 degrees
  // clockwise
  if (!wheelCounterClockwise)
  {
    step.moveTo(-1 * QUARTER_ROTATE);
    while (digitalRead(motorIndex) != LOW)
    {
      delayMicroseconds(10);
      
      if (!step.run())
      {
        wheelClockwise = false;
        break;
      }
    }
  }
  
  step.setCurrentPosition(0);
  if (wheelCounterClockwise)
  {
    step.moveTo(QUARTER_ROTATE);
  }
  else
  {
    step.moveTo(-1 * QUARTER_ROTATE);
  }
  while (digitalRead(motorIndex) == LOW)
  {
    delayMicroseconds(10);
    step.run();
  }
  numSteps = step.currentPosition();
  center = numSteps / 2;

  step.setCurrentPosition(0);
  step.moveTo(-1 * center);
  while (step.run()) 
  {
    delayMicroseconds(10);
  }
  step.setCurrentPosition(0);
}

void setup()
{  
  Serial.begin(9600);

  // Initialize the ros node handle and subscribe to the steering topic
  nh.initNode();
  nh.subscribe(sub);

  FL.setMaxSpeed(200);
  FL.setAcceleration(200);
  FL.setEnablePin(A2);
  pinMode(4, INPUT_PULLUP);
  
  FR.setMaxSpeed(200);
  FR.setAcceleration(200);
  FR.setEnablePin(A2);
  pinMode(7, INPUT_PULLUP);
  
  RL.setMaxSpeed(200);
  RL.setAcceleration(200);
  RL.setEnablePin(A2);
  pinMode(10, INPUT_PULLUP);
  
  RR.setMaxSpeed(200);
  RR.setAcceleration(200);
  RR.setEnablePin(A2);
  pinMode(A1, INPUT_PULLUP);
  
  calibrateMotor(FL, 4);
  //calibrateMotor(FR, 10);
  //calibrateMotor(RL, 13);
  //calibrateMotor(RR, 25);
}

void loop()
{  
  nh.spinOnce();
  delayMicroseconds(10);
}
