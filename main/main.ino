#include <Wire.h> // Allows communication with servo
#include <Adafruit_PWMServoDriver.h> // include the Servo library
#include <Stepper.h> // include the Stepper library

//Stepper Setup
#define STEPS 100 // steps per revolution of stepper motor
const int analogPin = 0; // analog in pin for stepper
const int baseSpeed = 50; // RPM of base stepper
// creating stepper objects to control steppers
Stepper base(STEPS, 7, 8, 9, 10); // assume stepper is connected to pins 7-10

const float minMaxArmLength[] = {50, 100}; // [mm] fully retracted/extended arm lengths
const float degPermm = 2; // how much the servo needs to turn to extend telescope by 1 mm
const float armAngleRatio = 5; // ratio of servo rotation:arm angle
const float baseAngleRatio = 10; // ratio of stepper rotation:base angle
const int gripperRatio = 100; // how much servo needs to rotate to open/close gripper

// initialize variables to store motor positions
int gripperPos = 300;   //ranges from 150 to 520
int telescopePos = 300; //ranges from 145 to 575
int armAnglePos = 300;  //ranges from 175 to 725
int basePos;

//Servo Setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int val[3] = {armAnglePos, telescopePos, gripperPos}; // Variable holds the value of each servo. Currently all set at a middle value
// use convention {armAngle, Telescope, Gripper} for the servos

// variables to set up spherical coords
float theta = 0; // [deg] base angle
float phi = 0; // [deg] arm angle (assume starting in upright pos - z axis)
float armLength = minMaxArmLength[0]; // assume arm is initially fully retracted
int gripperState = 0; // 0 for closed, 1 for open

String valInput; //for testing
int i=0;
//coordinated stored here
int pick[2] = {0, 0}; 
int place[2] = {0, 0};

void setup() {
  base.setSpeed(baseSpeed);

  basePos = analogRead(analogPin);

  Serial.begin(9600); // initialize the serial port
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  Serial.println("Running example: Servo motor actuation using messaging");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  //Set servo to starting values
  Serial.print(" Servo values = [ ");
  for (i=0; i<3; i++) {
    Serial.print(val[i]);
    Serial.print(" ");
  }
  Serial.print("]");
      
  //send values to servo
  for (i=0; i<3; i++) {
    pwm.setPWM(i+1, 0, val[i]);
    }
}

 
void moveBaseAngle(float angle) { // [deg] rotate base
  theta = theta + angle; // update coords
  angle = angle * baseAngleRatio; // convert to stepper angles

  int current = analogRead(analogPin); // current step
  basePos = basePos + round(angle / 360 * STEPS); // target step
  base.step(basePos - current);
}

void moveArmAngle(float angle) { // [deg] rotate arm
  phi = phi + angle; // update coords
  //removed for the test, currently recieved angle is the change in PWM signal
  //angle = angle * armAngleRatio; // convert to servo angles

  armAnglePos = armAnglePos + angle;

  //Move servo
  for (i=0; i<abs(angle)+10; i++){
    if (val[0] < armAnglePos){
      val[0] = val[0]+1;
    }
    else if (val[0] > armAnglePos) {
      val[0] = val[0]-1;
    }
    pwm.setPWM(1, 0, val[0]);
    delay(100);
  }
}

void moveTelescope(float dist) { // [mm] extend/retract arm
  armLength = armLength + dist; // update coords
  //removed for the test, currently recieved dist is the change in PWM signal
  //dist = dist * degPermm; // convert to servo angles

  telescopePos = telescopePos + dist;

  //Move Servo
  for (i=0; i<abs(dist)+10; i++){
    if (val[1] < telescopePos){
      val[1] = val[1]+1;
    }
    else if (val[1] > telescopePos) {
      val[1] = val[1]-1;
    }
    pwm.setPWM(2, 0, val[1]);
    delay(100);
  }
}

void moveGripper(int state) { // 0 to close, 1 to open
  if (state == 0 && state != gripperState) { // closing gripper
    gripperState = state; //Update gripper state
    gripperPos = gripperPos - gripperRatio; // assuming subtract for closing (flip above if needed)
  } else if (state == 1 && state != gripperState) { // opening gripper
    gripperState = state; //Update gripper state
    gripperPos = gripperPos + gripperRatio;
  }
  //Move Servo
  for (i=0; i<abs(gripperRatio)+10; i++){
    if (val[2] < gripperPos){
      val[2] = val[2]+1;
    }
    else if (val[2] > gripperPos) {
      val[2] = val[2]-1;
    }
    pwm.setPWM(3, 0, val[2]);
    delay(100);
  }
}

bool validMove(float newTheta, float newPhi, float newArmLength) { // TODO: validate if moving to new coords is possible
  return true;
}

void loop() {
  // TODO: put your main code here, to run repeatedly:
  
  //Test
  if (Serial.available() > 0) {
    valInput = Serial.readString();
    Serial.print("I recived: ");
    Serial.print(valInput);

    //Uncomment any of the code below to test each servo
    //moveArmAngle(valInput.toInt());
    //moveTelescope(valInput.toInt());
    moveGripper(valInput.toInt());
    
    //Print current servo vales
    Serial.print(" Servo values = [ ");
    for (i=0; i<3; i++) {
      Serial.print(val[i]);
      Serial.print(" ");
    }
    Serial.print("]");
        
    //send values to servo
    for (i=0; i<3; i++) {
      pwm.setPWM(i+1, 0, val[i]);
      }
  }//test ends

/* Main code here
 *  
 *  
 *  standy (waiting for coordinates of pickup)
 *  
 *  recive coordinated from pi.
 *  pick[1] = ...
 *  pick[2] = ...
 *  place[1]= ...
 *  place[2]= ...
 *   
 *  Check if they are valid moves (Optional)
 * 
 * Pick
 *  rotate base:        moveBaseAngle(pick[1])
 *  extend arm:         moveTelescope(pick[2])
 *  lower arm:          moveArmAngle(0)
 *  grip test tube:     moveGripper(1)
 *  
 *  return to middle:   moveArmAngle(90)
 *                      moveTelescope(0)
 *                      moveBaseAngle(0)
 *  
 * Place
 *  rotate base:        moveBaseAngle(place[1])
 *  extend arm:         moveTelescope(place[2])
 *  lower arm:          moveArmAngle(0)
 *  release test tube:  moveGripper(0)
 *  
 *  return to middle:   moveArmAngle(90)
 *                      moveTelescope(0)
 *                      moveBaseAngle(0)
 *  
 *  
 *  
 *  Loop back to top
 *  
 *  
  
  */
}
