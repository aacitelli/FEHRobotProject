// FEH-Specific Libraries
#include <FEHLCD.H> // Necessary for during-test debug information
#include <FEHSD.h> // Necessary for post-test debug information
#include <FEHBattery.h> // Necessary for voltage check at beginning

// C/C++ Libaries
#include <cmath>
#include <stdlib.h>
#include <ctime>

// Custom Libraries
#include "CustomLibraries/constants.h"
#include "CustomLibraries/posttest.h"
#include "CustomLibraries/pretest.h"
#include "CustomLibraries/navigation.h"
#include "CustomLibraries/testing.h"

using namespace std;

void init(); void deinit();
void loopWhileStartLightIsOff();
void performanceTest4();
void finalRoutine();
void rpsTest();
void updateLastValidRPSValues();
void turnSouthAndGoUntilRPS(float startHeading);
void calibrate();

int main(void)
{
    // Initializes RPS & SD Card
    init();

    // Calibration procedure
    calibrate();

    // This is where we put the token in
    armServo.SetDegree(30);

    // This is our "final action"
    LCD.WriteLine("Waiting for final touch.");
    loopUntilTouch();

    // Sensing the start light, automatically triggering if it takes more than 30 seconds
    int iterationCount = 0;
    while (lightSensor.Value() > .75 && iterationCount < 300)
    {
        iterationCount++;

        clearLCD();
        LCD.WriteLine("Waiting for start light.");
        SD.Printf("Waiting for start light.\r\n");

        Sleep(.1);
    }

    // Runs any code from the current routine
    finalRoutine();

    // Shuts down whatever needs shut down at the end of a run
    deinit();

    // Exit code 0, indicating early termination did not occur
    return 0;
}

/**
 * @brief finalRoutine is the chain of goToPoint (and other misc. function) calls that make up our final competition run.
 */
void finalRoutine()
{
    /* Navigating to the token drop */
    // Approximate, Faster Positioning
    goToPoint(TOKEN_X - 4, TOKEN_Y - 3, false, 0.0, false, 0.0, false, 6);

    // More precise, slower positioning
    goToPoint(TOKEN_X, TOKEN_Y, true, TOKEN_HEADING, false, 0.0, false, 0);

    // Small wind-down time so that the next method run starts with an accurate heading
    Sleep(.2);

    // Turns slowly, but really precisely
    turnToAngleWhenAlreadyReallyClose(TOKEN_HEADING);

    // Dropping the token
    float currentDegree = 30;
    while (currentDegree <= 115)
    {
        armServo.SetDegree(currentDegree);
        currentDegree++;
        Sleep(.0075);
    }
    Sleep(.5);
    armServo.SetDegree(30);
    Sleep(.5);

    // Go to the side of one of the lights so that we can correctly align onto the close button
    goToPoint(16, 15, false, 0.0, false, 0.0, false, 6);

    // Go on top of the near light
    goToPoint(DDR_BLUE_LIGHT_X - 4.25, DDR_LIGHT_Y, false, 0.0, false, 0.0, false, 2);

    // Reading light sensor output
    leftMotor.Stop();
    rightMotor.Stop();
    SD.Printf("Light Sensor Output: %f\r\n", lightSensor.Value());

    // If the light is blue, do this pathfinding and press the blue button
    if (lightSensor.Value() > 1.0)
    {
        // Positioning approximately above the blue button
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 5, true, 270, false, 0.0, false, 3);

        // Give first tolerance check in next function time to catch up (had minor issues w/ this otherwise, so this is here as insurance)
        Sleep(.4);

        // Was having consistency issues with being straight enough, so this one should reduce the angle we're currently at from like += 10 degrees to += 5
        turnToAngleWhenKindaClose(270);

        // Driving down into the blue button
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y - 5, false, 0.0, true, 22.0, false, 1);
    }

    // Otherwise, the light is red, so do red button pathfinding and press the red button
    else
    {
        // Positioning above button
        goToPoint(DDR_BLUE_LIGHT_X - 4.25, DDR_LIGHT_Y + 5, true, 270, false, 0.0, false, 3);

        // See above note
        Sleep(.4);

        // Was having consistency issues with being straight enough, so this one should reduce the angle we're currently at from like += 10 degrees to += 5
        turnToAngleWhenKindaClose(270);

        // Hitting button for long enough to get bonus goal too
        goToPoint(DDR_BLUE_LIGHT_X - 4.25, DDR_LIGHT_Y - 5, false, 0.0, true, 22.0, false, 1);

        // In the case of red, we have to back up a little bit so that we don't turn into the blue button when aligning for the RPS button
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 5, true, 90, false, 0.0, true, 2);
    }

    // Space and angle for the RPS button
    goToPoint(RPS_BUTTON_X, RPS_BUTTON_Y, true, RPS_BUTTON_HEADING, false, 0.0, false, 0);

    // Giving goToPoint time to "wind down motors"
    Sleep(.2);

    // Making sure the angle for the RPS button is super accurate
    turnToAngleWhenAlreadyReallyClose(RPS_BUTTON_HEADING);

    // Physically pressing the RPS button
    armServo.SetDegree(125); Sleep(4.0);
    armServo.SetDegree(30);

    // So that the robot turns right to get to the bottom of the ramp and not the left (where it runs the risk of hitting the blue button)
    turn(90);

    // Move to bottom of ramp
    goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 2, false, 0.0, false, 0.0, false, 5);

    // Move up ramp and stop somewhere near the top nearish to foosball
    // TODO - Add an additional checkpoint here so that it doesn't occasionally catch
    goToPoint(DDR_BLUE_LIGHT_X + 2, 40, false, 0.0, false, 0.0, false, 5);
    goToPoint(DDR_BLUE_LIGHT_X + 1.8, 57, false, 0.0, false, 0.0, false, 5);

    // Past this point, this check needs to be here for basically every call so if it loses deadzone it skips all the way to the end
    if (!hasExhaustedDeadzone)
    {
        // Positions for foosball itself
        goToPoint(FOOSBALL_START_X, FOOSBALL_START_Y - .25, true, 7.0, false, 0.0, false, 2);

        // Makes sure the motors are caught up so that the specific angle check is as accurate as possible
        Sleep(.3);

        // Turning really specifically to the angle
        turnToAngleWhenAlreadyReallyClose(6);
    }

    // Pressing down on the counters, giving the servo time to get down
    armServo.SetDegree(95);
    Sleep(.5);

    // The "going backwards" part of foosball
    if (!hasExhaustedDeadzone)
    {
        // Physically pulling the counters over
        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .4);
        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .4);
        Sleep(1.9);

        // Stopping the motors
        leftMotor.Stop();
        rightMotor.Stop();

        // Lifting the arm off of the counters
        armServo.SetDegree(75);
        // Sleep(.5); // Put this back in if it pulls the counters too far forward again at the end

        // Moving forward a little bit
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .2);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .2);
        Sleep(1.0);

        // Stopping the motors
        leftMotor.Stop();
        rightMotor.Stop();

        // Pressing the arm onto the counters again
        armServo.SetDegree(95);
        Sleep(.5);

        // Pulling the counters back again just to be sure
        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .2);
        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .2);
        Sleep(1.0);

        // Stopping the motors
        leftMotor.Stop();
        rightMotor.Stop();

        // Rotating the arm off of the motors
        armServo.SetDegree(30);
        Sleep(.5);

        // Only do this if we don't make the robot go above the dodecahedron
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .5);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .5);
        Sleep(1.0);
        leftMotor.Stop();
        rightMotor.Stop();
    }

    // Going to the left part
    if (!hasExhaustedDeadzone)
        goToPoint(20, 48, false, 0.0, false, 0.0, false, 6);

    if (!hasExhaustedDeadzone)
        goToPoint(8, 48, false, 0.0, false, 0.0, false, 6);

    // Positioning for the lever
    // Approximate, faster positioning most of the way there
    if (!hasExhaustedDeadzone)
        goToPoint(LEVER_X + 1, LEVER_Y - 4, false, 0.0, false, 0.0, false, 5);

    // Positioning for the lever
    // More precise, slower positioning once we're nearly there
    if (!hasExhaustedDeadzone)
        goToPoint(LEVER_X, LEVER_Y, true, LEVER_HEADING, false, 1.5, false, 0);

    // Making sure tolerance check in next called function is very accurate
    Sleep(.4);

    // Turning really precisely to the lever
    if (!hasExhaustedDeadzone)
        turnToAngleWhenAlreadyReallyClose(LEVER_HEADING);

    // Pressing the lever
    armServo.SetDegree(105);
    Sleep(1.0);

    leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .4);
    rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .4);
    Sleep(.2);
    armServo.SetDegree(30);

    Sleep(.5);

    leftMotor.Stop();
    rightMotor.Stop();

    /* It skips to right here if RPS drops */
    // Approximately centered somewhere in front of the ramp
    goToPoint(6, 55.0, false, 0.0, false, 0.0, false, 6);

    // Approximately the end button
    goToPoint(5.5, 5.0, false, 0.0, false, 0.0, false, 6);
}


