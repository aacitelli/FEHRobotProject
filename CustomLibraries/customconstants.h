/* About This File:
 *
 *  This is a header file that contains the global variables for all the stuff that is plugged into the Proteus. This
 *  includes motors, bump switches, optosensors, and anything else that has a physical connection to the Proteus.
 *
 *  This header file is necessary so that all the other header files can have access to these variables. If they were
 *  to be declared in the main file, they would need to be passed into every function.
 *
 *  This file is imported into our main file near the top, and we can use any of these functions as if they were located
 *  in the main file.
 *
 *  This file is separate because these functions took up a lot of space. Separating methods that are part of the same
 *  sort of system (movement, in this case) makes everything easier and more clear to navigate to.
 *
 *  */

#ifndef CUSTOMCONSTANTS_H
#define CUSTOMCONSTANTS_H

#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>
#include <ctime>


// The Two Motors (used to turn the two wheels in the front)
FEHMotor leftMotor(FEHMotor::Motor0, 9.0);
FEHMotor rightMotor(FEHMotor::Motor1, 9.0);

// The Servo (used to rotate the arm)
FEHServo armServo(FEHServo::Servo0);

// Bump switches (used mostly for hardcoded performance test elements)
// Todo - Update these values
DigitalInputPin leftBumpSwitch(FEHIO::P0_3);
DigitalInputPin rightBumpSwitch(FEHIO::P0_3);

// The Light Sensor (Used to sense DDR color and course start)
AnalogInputPin lightSensor(FEHIO::P0_0);

// Motor Percentages
const int LEFT_MOTOR_SIGN_FIX = 1;
const int RIGHT_MOTOR_SIGN_FIX = 1;

// Left Clockwise, Right CCW
const float DEFAULT_MOTOR_PERCENT = 40;

const float LEFT_MOTOR_PERCENT = LEFT_MOTOR_SIGN_FIX * DEFAULT_MOTOR_PERCENT;
const float RIGHT_MOTOR_PERCENT = RIGHT_MOTOR_SIGN_FIX * DEFAULT_MOTOR_PERCENT;

const float SLOWER_LEFT_MOTOR_PERCENT = LEFT_MOTOR_PERCENT / 2;
const float SLOWER_RIGHT_MOTOR_PERCENT = RIGHT_MOTOR_PERCENT / 2;

// Light Thresholds
const float START_LIGHT_MIN_VALUE = 0;
const float START_LIGHT_MAX_VALUE = .5;

// Frequently Used Headings
const float NORTH = 90;
const float EAST = 0;
const float SOUTH = 270;
const float WEST = 180;

bool hasExhaustedDeadzone = false;

// Ending contents of header file
#endif
