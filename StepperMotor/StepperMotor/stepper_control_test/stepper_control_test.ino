#include <Servo.h>
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

AccelStepper FL_(AccelStepper::DRIVER, 3, 2);
AccelStepper FR_(AccelStepper::DRIVER, 6, 5);
AccelStepper RL_(AccelStepper::DRIVER, 9, 8);
AccelStepper RR_(AccelStepper::DRIVER, 12, 11);

int numSteps;
int center;
int QUARTER_ROTATE = 220;

boolean wheelCounterClockwise = true;
boolean wheelClockwise = false;
boolean calibrating = true;

long motorPos[4];

void setup() {
    Serial.begin(28800);     // opens serial port, sets data rate to 9600 bps
    FL_.setMaxSpeed(500);
    FL_.setAcceleration(500);
    FL_.setEnablePin(A2);
    pinMode(4, INPUT_PULLUP);
    
    FR_.setMaxSpeed(500);
    FR_.setAcceleration(500);
    FR_.setEnablePin(A2);
    pinMode(7, INPUT_PULLUP);
    
    RL_.setMaxSpeed(500);
    RL_.setAcceleration(500);
    RL_.setEnablePin(A2);
    pinMode(10, INPUT_PULLUP);
    
    RR_.setMaxSpeed(500);
    RR_.setAcceleration(500);
    RR_.setEnablePin(A2);
    pinMode(A1, INPUT_PULLUP);
    
    calibrateMotor(FL_, 4);
    //calibrateMotor(FR_, 10);
    //calibrateMotor(RL_, 13);
    //calibrateMotor(RR_, 25);
}




void loop() {
   // serial message should consist of 4 ints that indicate the motor positions 
   // is the form of front left pos, front right pos, rear left pos, rear right pos
   // protocol follows <front_left angle>:<front_right angle>:<rear_left>:<reat_right>\n
   
    // send data only when you receive data:
    if (Serial.available() > 0) {
        //only should be called if a stepper message has been recieved

        //split the incoming message into its seperate motor components
        String stuff = Serial.readStringUntil('\n');
        messageSplitter(stuff, ':', motorPos);        
        
        //move the front left stepper to the first int of a message
        //Serial.print("front left motor position: ");
        Serial.println(motorPos[0]);
        FL_.moveTo(motorPos[0]);
        
        //move the front right stepper to the second int of a message
       // Serial.print("front right motor position: ");
        //Serial.println(motorPos[1]);
        //FL_.moveTo(motorPos[1]);

        //move the Rear left stepper to the third int of a message
       // Serial.print("Rear Left motor position: ");
       // Serial.println(motorPos[2]);
        //FL_.moveTo(motorPos[2]);

        //move the Rear right stepper to the forth int of a message
       // Serial.print("Rear right motor position: ");
       // Serial.println(motorPos[3]);
        //FL_.moveTo(motorPos[3]);
            
    }
    if (!calibrating) {
       FL_.run();
      //FR_.run();
      //RL_.run();
      //RR_.run();
    }
   
}

void messageSplitter(String message, char splitVar, long* positions){
  int splitIndex = 0;
  String temp = "";
  for (int i = 0; i < message.length() - 1; i++){
    if (message.charAt(i) == splitVar){
      positions[splitIndex] = temp.toInt();
      temp = "";
      splitIndex++;
    } else {
      temp += message.charAt(i);
    }
    
  }
  positions[splitIndex] = temp.toInt();
}

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
  Serial.println("finished Calibration");
  calibrating = false;
}
