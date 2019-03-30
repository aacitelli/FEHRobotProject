#ifndef CUSTOMCONSTANTS_H
#define CUSTOMCONSTANTS_H

// Imports
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>

// No custom library imports needed (screws up all sorts of things, anyways)

// I/O 
FEHMotor leftMotor(FEHMotor::Motor0, 9.0);
FEHMotor rightMotor(FEHMotor::Motor1, 9.0);
FEHServo armServo(FEHServo::Servo0);
DigitalInputPin leftBumpSwitch(FEHIO::P0_3);
DigitalInputPin rightBumpSwitch(FEHIO::P0_3);
AnalogInputPin lightSensor(FEHIO::P0_0);

// Calibration values for RPS pathfinding
float TOKEN_X, TOKEN_Y, TOKEN_HEADING;
float DDR_BLUE_LIGHT_X, DDR_LIGHT_Y;
float RPS_BUTTON_X, RPS_BUTTON_Y, RPS_BUTTON_HEADING;
float FOOSBALL_START_X, FOOSBALL_START_Y, FOOSBALL_END_X, FOOSBALL_END_Y;
float LEVER_X, LEVER_Y, LEVER_HEADING;

// Motor Percentages & Sign Fixes
const int LEFT_MOTOR_SIGN_FIX = 1;
const int RIGHT_MOTOR_SIGN_FIX = 1;
const float DEFAULT_MOTOR_PERCENT = 100;
const float LEFT_MOTOR_PERCENT = LEFT_MOTOR_SIGN_FIX * DEFAULT_MOTOR_PERCENT;
const float RIGHT_MOTOR_PERCENT = RIGHT_MOTOR_SIGN_FIX * DEFAULT_MOTOR_PERCENT;
float currentLeftMotorPercent = -1;
float currentRightMotorPercent = -1;

// Cardinal Headings 
const float NORTH = 90;
const float EAST = 0;
const float SOUTH = 270;
const float WEST = 180;

// Values kept to allow the robot to semi-intelligently get back 
// to RPS when deadzone hits and the robot is still in it
// Todo - Update these two variables, because I have no idea what motor speed (maybe 40?) I was at when I initially calculated these
// Current variable values mean it went 690 Degrees in 5 seconds, just for reference 
const float DEGREES_PER_SECOND = (690 / 5.0);
const float SECONDS_PER_DEGREE = (5.0 / 690);
bool hasExhaustedDeadzone = false;
float lastValidX, lastValidY, lastValidHeading;

// Used to account for differences between RPS coordinates and where our robot turns around (the centroid) 
const float DISTANCE_BETWEEN_RPS_AND_CENTROID = 0;

// Misc. Mathematical Constants
#define PI 3.14159265358979323846

#endif
