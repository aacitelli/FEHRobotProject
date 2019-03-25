/* Todos (Unordered):
 *
 *  Note: Anything necessary for a performance test should be prioritized. Some of these are just for documentation purposes,
 *  and some are optional features that could make the robot faster or more reliable. Please do stuff relevant to the next performance test if possible.
 *
 *  - Implement timestamping at the beginning of all the SD card lines
 *  - Add a calibration procedure where we figure out the exact points that we need to go to and put the robot there beforehand
 *      - We could also just measure a single point then use that point to figure out how far the course is from the other ones, then adjust all measurements accordingly (ask me to explain this if you want to implement this; Hard to explain in text)
 *      - I think this is our best way of negating the small differences between the courses
 *  - Use dead-reckoning to figure out robot positioning whenever there are flickers in RPS (can compensate for maybe like half a second of RPS downtime, but helps us be way more precise with small flickers)
 *      - Would have to figure out exactly how fast the robot goes, but it's otherwise not hugely complicated and could make our robot better in spotty rps areas like the ramp
 *  - Implement a timeout for the robot starting (maybe like 15 seconds after the RPS check) to avoid a dead robot should the initial trigger fail (which is very, very rare, but worth putting in due to how easy it is to do and how bad it would be if we missed a light check)
 *  - Implement verbosity levels for error reporting (e.g. 1 = Verbose (Everything), 2 = Intermediate (Unintended method behavior, big method results), 3 = Error (Only unintended method behavior output))
 *      - This would actually be pretty helpful. You can do it by using the above scale and wrapping every debug statement in an if(verbosity constant we hardcode in is > the threshold for reporting this) or similar
 *
 * */

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

using namespace std;

void init(); void deinit();
void loopWhileStartLightIsOff();
void performanceTest4();
void rpsTest();
void updateLastValidRPSValues();
void turnSouthAndGoUntilRPS(float startHeading);

int main(void)
{
    init();

    while (lightSensor.Value() > .45)
    {
        LCD.WriteLine("Waiting for light to turn on.");
        Sleep(.1);
        clearLCD();
    }

    performanceTest4();

    deinit();

    // Indicates that the program was successfully run
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
        Sleep(.1);
    }
}

float lastValidX, lastValidY, lastValidHeading;

void finalRoutine()
{

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
    goToPoint(0.0, 0.0, 0.0, fdswq2alse, 2.0, 1.0);
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
