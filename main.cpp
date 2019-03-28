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
void calibrate();

int main(void)
{
    leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .5);
    rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .5);
    Sleep(3.0);

    leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .5);
    rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .5);
    Sleep(3.0);

    init();
    calibrate();

    // Our "final action" is pressing the button or w/e to start the course itself
    // 300 -> Amount of iterations in 30 seconds (max time between start and w/e)
    /*
    int iterationCount = 0;
    while (lightSensor.Value() > .45)
    {
        LCD.WriteLine("Waiting for light to turn on.");
        Sleep(.1);
        clearLCD();

        iterationCount++;
        if (iterationCount > 300)
            break;
    }
    */

    Sleep(10.0);

    // Runs any code from the current routine
    finalRoutine();

    // Shuts down whatever needs shut down at the end of a run
    deinit();

    // Return value of zero indicates that no unrecoverable errors occurred
    return 0;
}

// Calibration values for RPS pathfinding
float TOKEN_X, TOKEN_Y, TOKEN_HEADING;
float DDR_BLUE_LIGHT_X, DDR_LIGHT_Y;
float RPS_BUTTON_X, RPS_BUTTON_Y, RPS_BUTTON_HEADING;
float FOOSBALL_START_X, FOOSBALL_START_Y, FOOSBALL_END_X, FOOSBALL_END_Y;
float LEVER_X, LEVER_Y, LEVER_HEADING;

void calibrate()
{
    // Holds user touch positions
    float x, y;

    // Token
    while (!LCD.Touch(&x, &y))
    {
        clearLCD();
        LCD.WriteLine("Waiting for Token Positioning.");
        Sleep(.1);
    }

    loopWhileNoRPS();
    TOKEN_X = RPS.X();
    TOKEN_Y = RPS.Y();
    TOKEN_HEADING = RPS.Heading();

    Sleep(1.0);

    // DDR, Far Button
    while (!LCD.Touch(&x, &y))
    {
        clearLCD();
        LCD.WriteLine("Waiting for DDR Far Positioning.");
        Sleep(.1);
    }

    loopWhileNoRPS();
    DDR_BLUE_LIGHT_X = RPS.X();
    DDR_LIGHT_Y = RPS.Y();

    Sleep(1.0);

    // RPS Button
    while (!LCD.Touch(&x, &y))
    {
        clearLCD();
        LCD.WriteLine("Waiting for RPS Button Positioning.");
        Sleep(.1);
    }

    loopWhileNoRPS();
    RPS_BUTTON_X = RPS.X();
    RPS_BUTTON_Y = RPS.Y();
    RPS_BUTTON_HEADING = RPS.Heading();

    Sleep(1.0);

    // Foosball Start
    while (!LCD.Touch(&x, &y))
    {
        clearLCD();
        LCD.WriteLine("Waiting for Foosball Start Positioning.");
        Sleep(.1);
    }

    loopWhileNoRPS();
    FOOSBALL_START_X = RPS.X();
    FOOSBALL_START_Y = RPS.Y();

    Sleep(1.0);

    // Foosball End
    while (!LCD.Touch(&x, &y))
    {
        clearLCD();
        LCD.WriteLine("Waiting for Foosball End Positioning.");
        Sleep(.1);
    }

    loopWhileNoRPS();
    FOOSBALL_END_X = RPS.X();
    FOOSBALL_END_Y = RPS.Y();

    Sleep(1.0);

    // Lever
    while (!LCD.Touch(&x, &y))
    {
        clearLCD();
        LCD.WriteLine("Waiting for Lever Positioning.");
        Sleep(.1);
    }

    loopWhileNoRPS();
    LEVER_X = RPS.X();
    LEVER_Y = RPS.Y();
    LEVER_HEADING = RPS.Heading();

    clearLCD();
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
    goToPoint(TOKEN_X, TOKEN_Y, .75, .5, true, TOKEN_HEADING, false, 0.0);

    // Dropping the token
    armServo.SetDegree(120);
    Sleep(1.0);
    armServo.SetDegree(30);

    // Go to the side of one of the lights
    goToPoint(16, 15, 1.0, .6, false, 0.0, false, 0.0);

    // Go on top of the near light
    goToPoint(DDR_BLUE_LIGHT_X - 4.5, DDR_LIGHT_Y, .75, .4, false, 0.0, false, 0.0);

    leftMotor.Stop();
    rightMotor.Stop();

    // Deciding what color the light is and acting conditionally based on which case is true
    // If inconsistency is an issue, make this take samples over a second and take the average
    // Blue light is around .83, Red light is around .25 (Keep this line for reference)
    const float DDR_LIGHT_CUTOFF_VALUE = .5;

    // If the light is blue, do this pathfi`nding and press the blue button
    if (lightSensor.Value() > DDR_LIGHT_CUTOFF_VALUE)
    {
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 3, .75, .5, true, 270, false, 0.0); // Positioning above button
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y - 3, .75, .7, false, 0.0, true, 2.0); // Hitting button
    }

    // Otherwise, the light is red, so do red button pathfinding and press the red button
    else
    {
        goToPoint(DDR_BLUE_LIGHT_X - 4.5, DDR_LIGHT_Y + 3, .75, .5, true, 270, false, 0.0); // Positioning above button
        goToPoint(DDR_BLUE_LIGHT_X - 4.5, DDR_LIGHT_Y - 3, .75, .7, false, 0.0, true, 1.0); // Hitting button
    }

    // Space and angle for the RPS button
    goToPoint(RPS_BUTTON_X, RPS_BUTTON_Y, .5, .3, true, RPS_BUTTON_HEADING, false, 0.0);

    // Press the RPS button
    armServo.SetDegree(110); Sleep(4.0);
    armServo.SetDegree(30);

    // Move to bottom of ramp
    goToPoint(28.75, 16.75, 1.5, .6, false, 0.0, false, 0.0);

    // Move up ramp and stop somewhere near the top
    goToPoint(31, 55, 2.0, .7, false, 0.0, false, 0.0);

    // Positioning for the foosball task
    if (!hasExhaustedDeadzone)
        goToPoint(FOOSBALL_START_X, FOOSBALL_START_Y, .5, .5, true, 0.0, false, 0.0);

    armServo.SetDegree(110);
    Sleep(.5);

    // After the foosball task
    if (!hasExhaustedDeadzone)
        goToPointBackwards(FOOSBALL_END_X, FOOSBALL_END_Y, .5, .5, false, 0.0, false, 0.0);

    armServo.SetDegree(30);
    Sleep(1.0);

    // Position for the lever
    // Todo - Decide if we want to approach head-on or from the side (from the side would probably be more consistent but would require slightly more precise positioning, I guess
    if (!hasExhaustedDeadzone)
        goToPoint(LEVER_X, LEVER_Y, .75, .5, true, LEVER_HEADING, false, 1.5);

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
    goToPoint(6, 55.0, 2.0, 1.0, false, 0.0, false, 0.0);

    // Go down the ramp and in front of the end button
    goToPoint(6, 8.0, 2.0, 1.0, false, 0.0, false, 0.0);
}

// Initializing requisite systems
void init()
{
    RPS.InitializeTouchMenu();
    SD.OpenLog();
}

// Deinitializing certain systems
void deinit()
{
    SD.CloseLog();
}
