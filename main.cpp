// Use 1 course as baseline
// Offset course based on 4 sampled points

// FEH-Specific Libraries
#include <FEHLCD.h> // Necessary for during-test debug information
#include <FEHSD.h> // Necessary for post-test debug information
#include <FEHBattery.h> // Necessary for voltage check at beginning

// C/C++ Libaries
#include <cmath>
#include <stdlib.h>

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

void miscTesting()
{
}

int main(void)
{
    init();

    // Our "final action" is pressing the button or w/e to start the course itself
    while (lightSensor.Value() > .45)
    {
        LCD.WriteLine("Waiting for light to turn on.");
        Sleep(.1);
        clearLCD();
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

float lastValidX, lastValidY, lastValidHeading;

void finalRoutine()
{
    // Navigating to the token drop
    goToPoint(13.4, 24.4, 23.3, true, .5, .75);

    // Dropping the token
    armServo.SetDegree(120);
    Sleep(1.0);
    armServo.SetDegree(30);

    // Navigating to DDR, going right above the left light
    goToPoint(23.5, 16, 90, true, .5, .75);

    // Backs up onto the light, going until it reaches a certain RPS threshold, keeping itself straight
    // Todo - Modify this to use backwards goToPoint
    leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .5);
    rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .5);

    // Light itself is at roughly y = 12.9, so cut this loop of very slightly early
    while (RPS.Y() > 13 || RPS.Y() == -1)
    {
        // If it's turned too far to the left, have it turn slightly right by having the right motor go a little harder
        if (RPS.Heading() > 95 && RPS.Heading() < 270)
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .5);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .6);
        }

        // If it's turned too far to the right, have it turn slightly left but having the left motor go a little harder
        else if (RPS.Heading() < 95 || (RPS.Heading() < 360 && RPS.Heading() >= 270))
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .6);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .5);
        }

        // Otherwise, it should keep going straight
        else
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .5);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .5);
        }
    }

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
        goToPoint(28.6, 16, 270, true, .5, .5);

        // Calling goToPoint's backwards counterpart to go towards the button
        // Y-Coordinate must be far enough that it never passes the tolerance check but close enough that it is accurate enough to accurately pathfind
        goToPointBackwards(28.6, 12.0, 0.0, false, .5, .5);
    }

    // Otherwise, the light is red, so do red button pathfinding and press the red button
    else
    {
        // Getting set up above the red stuff
        goToPoint(23.5, 16, 270, true, .5, .5);

        // Calling goToPoint's backwards counterpart to go towards the button
        // Y-Coordinate must be far enough that it never passes the tolerance check but close enough that it is accurate enough to accurately pathfind
        goToPointBackwards(23.5, 12.0, 0.0, false, .5, .5);
    }

    // Todo - Stop taking points as a percentage of 40% and redo it to take them as a full percentage
    // Move to bottom of ramp
    goToPoint(28.75, 16.75, 0.0, false, 1.0, 1.0);

    // Move up ramp and stop somewhere near the top
    goToPoint(31, 55, 0.0, false, 1.0, 1.0);

    // Positioning for the foosball task
    goToPoint(22.75, 63.6, 0.0, true, .5, .4);

    // Todo - Implement some sort of deadzone cutoff that automatically turns as accurately as possible to south and goes that direction, then paths to the end button from there

    // Do the foosball task
    // Todo - Implement this

    // Position for the lever

    // Pull the lever

    // Navigate to the left edge of the course in preparation for the left ramp

    // Go down the left ramp

    // Press the end button
}

void performanceTest4()
{
    goToPoint(15, 18, 0.0, false, 3.0, 1.0);

    // This was 25.25 and 16.75 before, just incase this proves to be too inaccurate
    goToPoint(24.75, 16.6, 225.0, true, .3, .5);

    armServo.SetDegree(110);
    Sleep(4.0);

    armServo.SetDegree(30);
    Sleep(1.0);

    // Move to bottom of ramp
    goToPoint(28.75, 16.75, 0.0, false, 1.0, 1.0);

    // Move up ramp and stop somewhere near the top
    goToPoint(31, 55, 0.0, false, 1.0, 1.0);

    // Positioning for the foosball task
    goToPoint(22.75, 63.6, 0.0, true, .5, .4);

    armServo.SetDegree(110);
    Sleep(1.0);

    // The foosball task
    while ((RPS.X() > 15 || RPS.X() == -1) && RPS.X() != -2)
    {
        // If it needs to turn left to keep itself straight
        if (RPS.Heading() < 355.0 && RPS.Heading() > 310.0)
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .9);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .9);
        }

        // Otherwise, if it needs to turn right to keep itself straight
        else if (RPS.Heading() > 5.0 && RPS.Heading() < 50.0)
        {
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .9);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .9);
        }

        // Otherwise, just keep going straight because the heading is alright
        else
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .75);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .75);
        }

        LCD.Write("Current X: "); LCD.WriteLine(RPS.X());
        LCD.Write("Current Y: "); LCD.WriteLine(RPS.Y());
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());

        SD.Printf("DURING FOOSBALL: \r\n");
        SD.Printf("Current X: %f\r\n", RPS.X());
        SD.Printf("Current Y %f\r\n", RPS.Y());
        SD.Printf("Current Heading: %f\r\n", RPS.Heading());

        Sleep(.001);

        // Keeps track of the last valid RPS value so that we can intelligently(ish) turn south if something goes wrong
        updateLastValidRPSValues();
    }

    // Resetting arm servo to a degree that won't bump into anything
    armServo.SetDegree(30);
    Sleep(1.0);

    // If it lost RPS in a deadzone, turn as close to south as you can get and go until you get RPS
    if (RPS.X() == -2)
        turnSouthAndGoUntilRPS(lastValidHeading);

    // Go to (very approximately) the top of the ramp
    goToPoint(6, 55.0, 0.0, false, 2.0, 1.0);

    // Go down the ramp and in front of the end button
    goToPoint(6, 8.0, 225, true, 2.0, 1.0);

    // Press the end button
    goToPoint(0.0, 0.0, 0.0, false, 2.0, 1.0);
}

void turnSouthAndGoUntilRPS(float startHeading)
{
    leftMotor.SetPercent(LEFT_MOTOR_PERCENT);
    rightMotor.SetPercent(RIGHT_MOTOR_PERCENT);

    Sleep(.5);

    leftMotor.Stop();
    rightMotor.Stop();

    leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .5);
    rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .5);

    Sleep(2.0);

    leftMotor.Stop();
    rightMotor.Stop();

    leftMotor.SetPercent(LEFT_MOTOR_PERCENT);
    rightMotor.SetPercent(RIGHT_MOTOR_PERCENT);

    while (RPS.X() == -1 || RPS.X() == -2) { Sleep(.001); }
    Sleep(.5);
    leftMotor.Stop();
    rightMotor.Stop();


    /*
    // First Quadrant - More Turning Needed
    if (startHeading >= 0 && startHeading < 90)
    {
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT);
        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT);

        Sleep(1.2);
    }

    // Second Quadrant - More Turning Needed
    else if (startHeading >= 90 && startHeading < 180)
    {
        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT);

        Sleep(1.2);
    }

    // Third Quadrant - Less Turning Needed
    else if (startHeading >= 180 && startHeading < 270)
    {
        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT);

        Sleep(.4);
    }

    // Fourth Quadrant - Less Turning Needed
    else
    {
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT);
        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT);

        Sleep(.4);
    }

    leftMotor.Stop();
    rightMotor.Stop();

    Sleep(.5);

    leftMotor.SetPercent(LEFT_MOTOR_PERCENT);
    rightMotor.SetPercent(RIGHT_MOTOR_PERCENT);

    while (RPS.X() == -1 || RPS.X() == -2) { Sleep(.001); }

    // Give it a little leeway to make sure it's truly out of the deadzone
    Sleep(.5);

    leftMotor.Stop();
    rightMotor.Stop();
    */
}

// Updates global variables, but only to "valid" vales (anything that's not "no rps" or a deadzone value
void updateLastValidRPSValues()
{
    if (RPS.X() != -1 && RPS.X() != -2)
        lastValidX = RPS.X();
    if (RPS.Y() != -1 && RPS.Y() != -2)
        lastValidY = RPS.Y();
    if (RPS.Heading() != -1 && RPS.Heading() != -2)
        lastValidHeading = RPS.Heading();
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
