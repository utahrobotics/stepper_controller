#include <AccelStepper.h>

using namespace std;

/**
 * DESCRIPTION: This sketch is responsible for controlling the motors that steer the robot. The pin
 * assignments are as follows:
 *  D2: FL DIR    (Pin 5)
 *  D3: FL PUL    (Pin 6)
 *  D4: FL INDEX  (Pin 7)
 *  D5: FR DIR    (Pin 8)
 *  D6: FR PUL    (Pin 9)
 *  D7: FR INDEX  (Pin 10)
 *  D8: BL DIR    (Pin 11)
 *  D9: BL PUL    (Pin 12)
 *  D10: BL INDEX (Pin 13)
 *  D11: BR DIR   (Pin 14)
 *  D12: BR PUL   (Pin 15)
 *  A1: BR INDEX  (Pin 25)
 *  A2: EN        (Pin 24)
 */
 
AccelStepper FL(AccelStepper::DRIVER, 3, 2);
AccelStepper FR(AccelStepper::DRIVER, 6, 5);
AccelStepper BL(AccelStepper::DRIVER, 9, 8);
AccelStepper BR(AccelStepper::DRIVER, 12, 11);
int numSteps;
int center;
int QUARTER_ROTATE = 2000;    // TODO Find number of steps.
boolean wheelCounterClockwise = true;
boolean wheelClockwise = false;


/*
 * This method initializes the stepper's maximum speed and acceleration, sets the
 * enable pin, and calibrates the motor to determine its center.
 */
void calibrateMotor(AccelStepper step, int motorIndex)
{
  // Initialize maximums, enable pin, and the pull up for the index pin
  step.setMaxSpeed(200);
  step.setAcceleration(200);
  step.setEnablePin(A2);
  pinMode(motorIndex, INPUT_PULLUP);

  // Run the motor counter clockwise first to see if it can find the center
  step.moveTo(QUARTER_ROTATE);
  while (digitalRead(motorIndex) == LOW)
  {
    //Serial.println("Rotating CCW");
    delay(1);
    if (!step.run())
    { 
      wheelCounterClockwise = false;
      wheelClockwise = true;
      //Serial.println("Break loop 1");
      delay(1);
      break;
    }
  }
  //numSteps = step.currentPosition();
  
  // If the motor did not reach the full 45 degree turn, run the motor 90 degrees
  // clockwise
  if (!wheelCounterClockwise)
  {
    step.moveTo(-2 * QUARTER_ROTATE);
    while (digitalRead(motorIndex) == LOW)
    {
      // Serial.println("Rotating CW");
      delay(1);
      // step.run();
      if (!step.run())
      {
        wheelClockwise = false;
        break;
      }
    }
    
    if (!wheelClockwise && !wheelCounterClockwise)
    {
      //TODO Add error message if index pin never goes low.
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
  while (digitalRead(motorIndex) == HIGH)
  {
    //Serial.println("Counting steps");
    delay(1);
    step.run();
  }
  numSteps = step.currentPosition();
  center = numSteps / 2;

  step.setCurrentPosition(0);
  step.moveTo(-1 * center);
  while (step.run()) 
  {
    // Serial.println("Centering motor");
    delay(1);
    //step.run();
  }
  step.setCurrentPosition(0);
  Serial.println("End Calibration");
}

void setup()
{  
  Serial.begin(9600);
  Serial.println(digitalRead(4));
  //FL.setMaxSpeed(200);
  //FL.setAcceleration(200);
  //FL.setEnablePin(A2);
  
  calibrateMotor(FL, 4);
  // calibrateMotor(FR, 10);
  // calibrateMotor(BL, 13);
  // calibrateMotor(BR, 25);
  // FL.moveTo(QUARTER_ROTATE);
}

void loop()
{  
  
  
  //FL.run();
}
