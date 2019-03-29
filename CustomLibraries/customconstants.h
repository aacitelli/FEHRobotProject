// In short, tells the compiler to not make duplicates of this header file
#ifndef CUSTOMCONSTANTS_H
#define CUSTOMCONSTANTS_H

// Imports
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>

// I/O Mechanisms
FEHMotor leftMotor(FEHMotor::Motor0, 9.0);
FEHMotor rightMotor(FEHMotor::Motor1, 9.0);
FEHServo armServo(FEHServo::Servo0);
DigitalInputPin leftBumpSwitch(FEHIO::P0_3);
DigitalInputPin rightBumpSwitch(FEHIO::P0_3);
AnalogInputPin lightSensor(FEHIO::P0_0);

// Motor Percentages
const int LEFT_MOTOR_SIGN_FIX = 1;
const int RIGHT_MOTOR_SIGN_FIX = 1;
const float DEFAULT_MOTOR_PERCENT = 100;
const float LEFT_MOTOR_PERCENT = LEFT_MOTOR_SIGN_FIX * DEFAULT_MOTOR_PERCENT;
const float RIGHT_MOTOR_PERCENT = RIGHT_MOTOR_SIGN_FIX * DEFAULT_MOTOR_PERCENT;

// Frequently Used Headings
const float NORTH = 90;
const float EAST = 0;
const float SOUTH = 270;
const float WEST = 180;

// Used with hardcoded recovery function when deadzone hits
const float DEGREES_PER_SECOND = (690 / 5.0);
const float SECONDS_PER_DEGREE = (5.0 / 690);

// Todo - Path more intelligently around the dodecahedron instead of just trying to go south to make the panic algorithm more consistent

// Triggered if goToPoint is in a deadzone
// When triggered, causes the robot to skip all the goToPoint methods on the top level
// and try to path south using last remembered RPS values, get to RPS, and path with RPS to the end button
bool hasExhaustedDeadzone = false;
float lastValidX, lastValidY, lastValidHeading;

#endif
