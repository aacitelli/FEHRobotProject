// Use 1 course as baseline
// Offset course based on 4 sampled points

// FEH-Specific Libraries
#include <FEHLCD.h> // Necessary for during-test debug information
#include <FEHSD.h> // Necessary for post-test debug information
#include <FEHBattery.h> // Necessary for voltage check at beginning

// C/C++ Libaries
#include <cmath>
#include <stdlib.h>
#include <ctime>

// Custom Libraries
#include "CustomLibraries/customnavigation.h"
#include "CustomLibraries/customconstants.h"

/* Stuff I Want To Get Done This Week (Yes, Even Individual):
 *
 * HARDWARE (Loosely prioritized)
 *
 * - Implement wider surface areas across whatever we possibly can so that button pressing and anything else we need to press is as consistent as possible and works more consistently across all courses
 * - Make sure everything fits tight and nothing will come loose easily - Needs to be rigid and the motor setup has to be firm and not shaky (as close as we can get it)
 * - Try making the QR code anchored by a more than a single screw pivot (not sure how we'd do this, but consider it)
 * - Implement a valley system in the claw to help with our hardest task (foosball)
 *
 * SOFTWARE (Strictly prioritized with highest priority first and lowest priority last)
 *
 * - Implement code that does the course as consistently as possible - This is mostly just compiling code from our previous tests, but there's a bunch of new stuff too
 * - Implement RPS Standardization / Calibration Procedure Across All Courses (Helps avoid inconsistency issues)
 *      - This works by basically using one course as the "base" course and figuring out the average offset from the other courses
 *      - Going to have to ask a TA about the best way to do this
 * - Implement a timeout for robot starting (we had that one official performance test where it needed a little bump to get started
 * - Implement Dead-Reckoning (Guessing where the robot will be during downtime given last angle, position, and time)
 *      - This will require trig, measurements for how far the robot goes per time interval, and measurements for minor turn amounts
 *      - This is NOT intended as a serious replacement for deadzone navigation - This just covers up really small RPS inconsistencies (where RPS drops for less than a second)
 *      - This isn't really a priority to implement, but if done correctly, could bring really small overall improvements that could quickly compound
 *
 * */

using namespace std;

void init(); void deinit();
void loopWhileStartLightIsOff();
void performanceTest4();
void finalRoutine();
void rpsTest();
void updateLastValidRPSValues();
void turnSouthAndGoUntilRPS(float startHeading);

int main(void)
{
    init();

    // Our "final action" is pressing the button or w/e to start the course itself
    // 300 -> Amount of iterations in 30 seconds (max time between start and w/e)
    int iterationCount = 0;
    while (lightSensor.Value() > .45 && iterationCount < 300)
    {
        LCD.WriteLine("Waiting for light to turn on.");
        Sleep(.1);
        clearLCD();
        iterationCount++;
    }

    // Runs any code from the current routine
    finalRoutine();

    // Shuts down whatever needs shut down at the end of a run
    deinit();

    // Return value of zero indicates that no unrecoverable errors occurred
    return 0;
}

void rpsTest()
{
    while (true)
    {
        LCD.Clear();
        LCD.Write("X: "); LCD.WriteLine(RPS.X());
        LCD.Write("Y: "); LCD.WriteLine(RPS.Y());
        LCD.Write("Heading: "); LCD.WriteLine(RPS.Heading());
        Sleep(.2);
    }
}



// These points are points I went and measured on whatever course is on the closer half towards the consoles
void finalRoutine()
{
    // Navigating to the token drop
    // 13.4, x, 23.3 Before
    goToPoint(16, 24.4, 50.0, true, .5, .75, false, 0.0);

    // Dropping the token
    armServo.SetDegree(120);
    Sleep(1.0);
    armServo.SetDegree(30);

    // Go to the side of one of the lights
    goToPoint(16, 15, 0.0, false, .75, 1.5, false, 0.0);

    // Go on top of the near light
    goToPoint(24.25, 12.5, 0.0, false, .75, .75, false, 0.0);

    leftMotor.Stop();
    rightMotor.Stop();

    // Deciding what color the light is and acting conditionally based on which case is true
    // If inconsistency is an issue, make this take samples over a second and take the average
    // Blue light is around .83, Red light is around .25 (Keep this line for reference)
    const float DDR_LIGHT_CUTOFF_VALUE = .5;

    // If the light is blue, do this pathfi`nding and press the blue button
    if (lightSensor.Value() > DDR_LIGHT_CUTOFF_VALUE)
    {
        // Getting set up above the target position
        goToPoint(28.6, 16.5, 270, true, .75, .75, false, 0.0);

        // Pressing the button - Precision really matters
        goToPoint(29.5, 11, 270, false, .75, .5, true, 2.0);
    }

    // Otherwise, the light is red, so do red button pathfinding and press the red button
    else
    {
        // Getting set up above the red stuff
        goToPoint(23.5, 16.5, 270, true, .75, .75, false, 0.0);

        // Pressing the button - Precision really matters
        goToPoint(23.5, 10, 270, false, .75, .5, true, 2.0);
    }

    // Space and angle for the RPS button
    goToPoint(24.75, 16.6, 205.0, true, .5, .4, false, 0.0);

    // Press the RPS button
    armServo.SetDegree(110); Sleep(4.0);
    armServo.SetDegree(30);

    // Move to bottom of ramp
    goToPoint(28.75, 16.75, 0.0, false, 1.0, 1.5, false, 0.0);

    // Move up ramp and stop somewhere near the top
    goToPoint(31, 55, 0.0, false, 1.0, 1.5, false, 0.0);

    // Positioning for the foosball task
    if (!hasExhaustedDeadzone)
        goToPoint(22.75, 63.6, 0.0, true, .5, .5, false, 0.0);

    // Position for the lever
    // Todo - Decide if we want to approach head-on or from the side (from the side would probably be more consistent but would require slightly more precise positioning, I guess
    if (!hasExhaustedDeadzone)
        goToPoint(6, 55, 135, true, .75, .75, false, 1.5);

    // Pull the lever
    if (!hasExhaustedDeadzone)
    {
        armServo.SetDegree(110);
        Sleep(1.0);
        armServo.SetDegree(30);
    }

    /* Stuff from here down is taken directly from PT4 and should be pretty consistent, though the end buttoncould probably use some finesse */
    // Go to (very approximately) the top of the ramp
    // If it hits the deadzone, it should skip to here within a few seconds after it gets back to RPS
    goToPoint(6, 55.0, 0.0, false, 2.0, 1.0, false, 0.0);

    // Go down the ramp and in front of the end button
    goToPoint(6, 8.0, 225, true, 2.0, 1.0, false, 0.0);

    // Press the end button
    goToPoint(0.0, 0.0, 0.0, false, 2.0, 1.0, false, 0.0);
}





// Initializing requisite systems
void init()
{
    RPS.InitializeTouchMenu();
    SD.OpenLog();

    armServo.SetDegree(30);
}

// Deinitializing certain systems
void deinit()
{
    SD.CloseLog();
}
