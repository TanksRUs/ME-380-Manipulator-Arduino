#include <Servo.h> // include the Servo library
#include <Stepper.h> // include the Stepper library

// -----NEED TO CHANGE VALUES-----test
#define STEPS 100 // steps per revolution of stepper motor
const int gripperPin = 1; // gripper servo pin
const int telescopePin = 2; // telescoping servo pin
const int armAnglePin = 3; // arm angle servo pin
const int analogPin = 0; // analog in pin for stepper
const int baseSpeed = 50; // RPM of base stepper

const float minMaxArmLength[] = {50, 100}; // [mm] fully retracted/extended arm lengths
const float degPermm = 2; // how much the servo needs to turn to extend telescope by 1 mm
const float armAngleRatio = 5; // ratio of servo rotation:arm angle
const float baseAngleRatio = 10; // ratio of stepper rotation:base angle
const int gripperRatio = 50; // how much servo needs to rotate to open/close gripper

// creating stepper objects to control steppers
Stepper base(STEPS, 7, 8, 9, 10); // assume stepper is connected to pins 7-10

// creating servo objects to control servos
Servo gripper;
Servo telescope;
Servo armAngle;

// initialize variables to store motor positions
int gripperPos;
int telescopePos;
int armAnglePos;
int basePos;

// variables to set up spherical coords
float theta = 0; // [deg] base angle
float phi = 0; // [deg] arm angle (assume starting in upright pos - z axis)
float armLength = minMaxArmLength[0]; // assume arm is initially fully retracted
int gripperState = 0; // 0 for closed, 1 for open

void setup() {
  base.setSpeed(baseSpeed);

  // attach servos to respective pins
  gripper.attach(gripperPin);
  telescope.attach(telescopePin);
  armAngle.attach(armAnglePin);

  // read current motor positions
  gripperPos = gripper.read();
  telescopePos = telescope.read();
  armAnglePos = armAngle.read();
  basePos = analogRead(analogPin);

  Serial.begin(9600); // initialize the serial port
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
  angle = angle * armAngleRatio; // convert to servo angles

  armAnglePos = armAnglePos + angle;
  armAngle.write(armAnglePos);
}

void moveTelescope(float dist) { // [mm] extend/retract arm
  armLength = armLength + dist; // update coords
  dist = dist * degPermm; // convert to servo angles

  telescopePos = telescopePos + dist;
  telescope.write(telescopePos);
}

void moveGripper(int state) { // 0 to close, 1 to open
  if (state == 0 && state != gripperState) { // closing gripper
    gripperState = state;
    gripperPos = gripperPos - gripperRatio; // assuming subtract for closing (flip above if needed)
    gripper.write(gripperPos);
  } else if (state == 1 && state != gripperState) { // opening gripper
    gripperState = state;
    gripperPos = gripperPos + gripperRatio;
    gripper.write(gripperPos);
  }
}

bool validMove(float newTheta, float newPhi, float newArmLength) { // TODO: validate if moving to new coords is possible
  return true;
}

void loop() {
  // TODO: put your main code here, to run repeatedly:
}
