// FEH-Specific Libraries
#include <FEHLCD.H> // Necessary for during-test debug information
#include <FEHSD.h> // Necessary for post-test debug information
#include <FEHBattery.h> // Necessary for voltage check at beginning

// C/C++ Libaries
#include <cmath>
#include <stdlib.h>
#include <ctime>

// Custom Libraries
// Todo - Trim these down to what we actually need
#include "CustomLibraries/constants.h"
#include "CustomLibraries/conversions.h"
#include "CustomLibraries/navigation.h"
#include "CustomLibraries/posttest.h"
#include "CustomLibraries/pretest.h"
#include "CustomLibraries/rps.h"
#include "CustomLibraries/utility.h"

/* 

    Saturday Todo: 

    Know How To Do: 
    Test goToPoint autocorrection amounts (don't need to be on a course to do this) to make sure the turn radius is enough to fix any issue 
    Test turn autocorrection amounts to make sure it doesn't overshoot at all 

*/

/* NEXT WEEK'S TODO LISLT
 *
 * HARDWARE (Loosely prioritized)
 *
 * - Make sure everything relevant is covered in tape
 * - Make any aesthetic changes we want to (I'm 100% fine with going past 120 if it's for this)
 * - Re-affix the QR code - Several Mechanisms (Tape, Screw, Rubber Cement / Loctite)
 *
 * SOFTWARE (Highest Priority -> Lowest Priority)
 *
 * - Getting closer and closer to 100 points is the priority
 *      - Slight tuning to make it decently fast but at the same time precise enough to work consistently
 * - Clean up the codebase - It's already kind of organized, but NGL, this thing is kind of a mess
 * - Do documentation (/**) for every function in here
 *
 * MISC
 *
 * - All the website and documentation stuff
 * - Any robot assignments that are due
 * - Final competition reflection (I'm assuming that's due)
 *
 * */

// Todo - Write a really nice readme for the GitHub repo

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

    // Sensing the start light, automatically triggering if it takes more than 30 seconds
    int iterationCount = 0;
    while (lightSensor.Value() > .45 && iterationCount < 300)
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

// These points are points I went and measured on whatever course is on the closer half towards the consoles
void finalRoutine()
{
    // Navigating to the token drop
    // 13.4, x, 23.3 Before
    SD.Printf("Going to token.\r\n");
    goToPoint(TOKEN_X, TOKEN_Y, 1.0, .25, true, TOKEN_HEADING, false, 0.0, false);

    // Dropping the token
    SD.Printf("Depositing token.\r\n");
    armServo.SetDegree(120);
    Sleep(1.0);
    armServo.SetDegree(30);

    // Go to the side of one of the lights
    SD.Printf("Going to the side of one of the lights.\r\n");
    goToPoint(16, 15, 1.0, .25, false, 0.0, false, 0.0, false);

    // Go on top of the near light
    SD.Printf("Going on top of the near light.\r\n");
    goToPoint(DDR_BLUE_LIGHT_X - 4.5, DDR_LIGHT_Y, 1.0, .25, false, 0.0, false, 0.0, false);

    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(1.0);
    SD.Printf("Light Sensor Output: %f\r\n", lightSensor.Value());

    // If the light is blue, do this pathfinding and press the blue button
    if (lightSensor.Value() > .5)
    {
        clearLCD();
        LCD.WriteLine("BLUE.");
        SD.Printf("Light was detected as BLUE.\r\n");

        SD.Printf("Pathfinding above the blue light.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 5, .75, .25, true, 270, false, 0.0, false); // Positioning above button

        SD.Printf("Driving into the blue button.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y - 3, .75, .35, false, 0.0, true, 8.0, false); // Hitting button for long enough to get bonus goal too
    }

    // Otherwise, the light is red, so do red button pathfinding and press the red button
    else
    {
        clearLCD();
        LCD.WriteLine("RED.");
        SD.Printf("Light was detected as RED.\r\n");

        SD.Printf("Pathfinding above the red light.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X - 4.5, DDR_LIGHT_Y + 5, .75, .25, true, 270, false, 0.0, false); // Positioning above button

        SD.Printf("Driving into the red light.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X - 4.5, DDR_LIGHT_Y - 3, .75, .35, false, 0.0, true, 8.0, false); // Hitting button for long enough to get bonus goal too
    }

    // Getting a little bit of spacing from DDR before going to the end button
    SD.Printf("Pathfinding a little bit above DDR so we can go back to the end button.\r\n");
    goToPoint(DDR_BLUE_LIGHT_X - 2.0, DDR_LIGHT_Y + 3, 1.0, .3, false, 0.0, false, 0.0, false);

    // Go down the ramp and in front of the end button
    SD.Printf("Going down the ramp and into the end button.\r\n");
    goToPoint(5.0, 5.0, 1.5, .4, false, 0.0, false, 0.0, false);

    // Space and angle for the RPS button
    SD.Printf("Positioning for the RPS button.\r\n");
    goToPoint(RPS_BUTTON_X, RPS_BUTTON_Y, .4, .25, true, RPS_BUTTON_HEADING - 15, false, 0.0, false);

    // Press the RPS button
    SD.Printf("Pressing the RPS button.\r\n");
    armServo.SetDegree(110); Sleep(4.0);
    armServo.SetDegree(30);

    // Move to bottom of ramp
    SD.Printf("Positioning to move up the ramp\r\n");
    goToPoint(28, 16.75, .5, .25, false, 0.0, false, 0.0, false);

    // Move up ramp and stop somewhere near the top
    SD.Printf("Moving up the ramp.\r\n");
    goToPoint(28, 55, 1.5, .6, false, 0.0, false, 0.0, false);

    // Positioning for the foosball task
    if (!hasExhaustedDeadzone)
    {
        SD.Printf("Deadzone still negated. Positioning for foosball.\r\n");
        goToPoint(FOOSBALL_START_X, FOOSBALL_START_Y, .5, .5, true, 0.0, false, 0.0, false);
    }

    else
    {
        SD.Printf("Deadzone is in effect. Could not position initially for foosball.\r\n");
    }

    SD.Printf("Pressing down on foosball, even if a deadzone is still in effect.\r\n");
    armServo.SetDegree(110);
    Sleep(.5);

    // After the foosball task
    if (!hasExhaustedDeadzone)
    {
        SD.Printf("Deadzone still negated. Performing foosball.\r\n");
        goToPoint(FOOSBALL_END_X, FOOSBALL_END_Y, .5, .5, false, 0.0, false, 0.0, true);
    }

    else
    {
        SD.Printf("Deadzone is in effect. Could not actually go backwards while doing foosball.\r\n");
    }

    // This should be done regardless of if the deadzone is active or not
    SD.Printf("Foosball done. Raising arm back up so it doesn't get in the way.\r\n");
    armServo.SetDegree(30);
    Sleep(1.0);

    // Going to the left part
    if (!hasExhaustedDeadzone)
    {
        goToPoint(20, 48, 1.0, .5, false, 0.0, false, 0.0, false);
    }

    if (!hasExhaustedDeadzone)
    {
        goToPoint(8, 48, .5, 1.0, false, 0.0, false, 0.0, false);
    }

    // Position for the lever
    // Todo - Decide if we want to approach head-on or from the side (from the side would probably be more consistent but would require slightly more precise positioning, I guess
    if (!hasExhaustedDeadzone)
    {
        SD.Printf("Deadzone still negated. Positioning for lever.\r\n");
        goToPoint(LEVER_X, LEVER_Y, .75, .5, true, LEVER_HEADING, false, 1.5, false);
    }

    else
    {
        SD.Printf("Deadzone is back in effect. Could not position for lever.\r\n");
    }

    SD.Printf("Pressing lever, even if deadzone is now in effect.\r\n");
    armServo.SetDegree(110);
    Sleep(1.0);
    armServo.SetDegree(30);

    // If it hits the deadzone, it should skip to here within a few seconds after it gets back to RPS
    SD.Printf("Going to the top of the return ramp.\r\n");
    goToPoint(6, 55.0, 2.0, 1.0, false, 0.0, false, 0.0, false);
}


