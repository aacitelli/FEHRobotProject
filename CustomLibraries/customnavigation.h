/* About This File:
 *
 *  This is the custom-built "movement" library for our robot. It was living hell to program (mostly due to trig, trig sucks)
 *  and is probably also living hell to debug. God help us all.
 *
 *  It contains a very modular navigation where each function itself doesn't do much but can be combined very easily
 *  with others in order to make our entire movement system.
 *
 *  This file is imported into our main file near the top, and we can use any of these functions as if they were located
 *  in the main file.
 *
 *  This file is separate because these functions took up a lot of space. Separating methods that are part of the same
 *  sort of system (movement, in this case) makes everything easier and more clear to navigate to.
 *
 *  */

/**

 * Organization:
 *
 * Overall method used 99% of the time is goToPoint, which has access to the robot's X, Y, and Heading and has the intended X, Y, and Heading as arguments.
 *
 * First, it will turn forwards toward the point until it reaches within a very small (like 4-5 degree) tolerance in angle.
 * Then, it will move forward. If its current angle is pointed slightly away from the point it's headed to, it will turn towards it slightly to compensate until it's again pointed towards it.
 *
 * Whenever it gets to within a specified distance tolerance of the "end point" (1-2 inches), it will do a hard spot, then turn whichever way is best in order
 * to turn towards the end heading.
 *
 * */

/* Talk about today:
 *
 * Misc. Items
 *
 *  - Is a valley the best thing if our arm is actually tilted?
 *  - Consider rotating the arm to be straight 90 degrees - I think it would be better but I want your input
 *
 * Strategies for Foosball
 *
 *  - My Ideal Order:
 *      - Go up near foosball
 *      - Push the dodecahedron off towards the token machine so it doesn't get in the way when we go to the lever
 *      - Go to foosball 90 degrees off, get arm caught, go backwards using modified, backwards goToPoint
 *  - Do a more basic version of goToPoint where it just tries to align itself with a certain heading and go backwards a certain speed
 *  - Better to press down on top or try and get our arm stuck behind?
 *      - I personally think its better to press on top because we'll have to do it regardless to get the counters off the wall, might as well go all the way
 *
 */

#ifndef CUSTOMNAVIGATION_H
#define CUSTOMNAVIGATION_H

// Requisite Libraries
#include "customconstants.h" // Used to get references to the motors
#include "customutility.h" // Used for unit conversions and widely applicable functions

#include <FEHLCD.h>
#include <FEHSD.h>
#include <FEHRPS.h>
#include <time.h>

// Function Predeclarations
void turn(float endHeading);
void turn(float endX, float endY);
float getDesiredHeading(float x1, float y1, float x2, float y2);
bool shouldTurnLeft(float startHeading, float endHeading);
void turn180DegreesAway(float endHeading);
void turn180DegreesAway(float endX, float endY);
float rotate180Degrees(float degrees);
void turnSouthAndGoUntilRPS(float startHeading);
void updateLastValidRPSValues();

using namespace std;

float lastValidX, lastValidY, lastValidHeading;

#define GOTOPOINT_COUNTS_PER_SECOND 10

// Todo - Make a version of goToPoint that goes backwards (instead of trying to optimize for whether backwards is faster or not algorithmically - Hardcode whether it's forwards or backwards)
bool goToPointRPSIsBad() { return (RPS.X() == -1 || RPS.Y() == -1 || RPS.Heading() == -1 || RPS.X() == -2 || RPS.Y() == -2 || RPS.Heading() == -2); }
void goToPoint(float endX, float endY, float endHeading, bool shouldTurnToEndHeading, float distanceTolerance, float percentageOfFullSpeed, bool isTimed, float time)
{
    SD.Printf("goToPoint: Going to point (%f, %f) and End Heading %f.\r\n", endX, endY, endHeading);

    // Turn toward initial point, making sure we have accurate RPS data beforehand
    loopWhileNoRPS();
    turn(endX, endY);

    int iterationCount  = 0;

    SD.Printf("goToPoint: Entering distance tolerance check.\r\n");
    while (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > distanceTolerance)
    {
        // Check for deadzone and go south if it ever detects it, triggering something that makes it skip relevant goToPoint methods if it goes over time
        if (RPS.X() == -2 || RPS.Y() == -2 || RPS.Heading() == -2)
        {
            // Causes the program to skip certain goToPoint calls
            hasExhaustedDeadzone = true;

            // Does what you think it does
            turnSouthAndGoUntilRPS(lastValidHeading);

            // Escapes this call of goToPoint because it doesn't really have RPS any more
            return;
        }

        // Checks if the loop has gone over its timing (if a passed-in boolean value indicates that this should be checked for)
        if (isTimed)
        {
            iterationCount++;
            if (iterationCount > (time * GOTOPOINT_COUNTS_PER_SECOND))
            {
                break;
            }
        }

        // If RPS missed a cycle, keep going with the current command until RPS fixes itself (or becomes a deadzone, though that case is handled elsewhere)
        if (goToPointRPSIsBad())
        {
            Sleep(.001);
            continue;
        }

        // Doing this frequently to make sure it always has valid backups
        updateLastValidRPSValues();

        // Current iteration is a go, so calculate where exactly it's supposed to be going
        float desiredHeading = getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY);

        // Debug Output
        clearLCD();
        LCD.Write("Current Iteration Count: "); LCD.WriteLine(iterationCount);
        LCD.Write("Necessary It. Count: "); LCD.WriteLine(time * GOTOPOINT_COUNTS_PER_SECOND);
        LCD.Write("Current (x, y): ("); LCD.Write(RPS.X()); LCD.Write(", "); LCD.Write(RPS.Y()); LCD.WriteLine(")");
        LCD.Write("Intended (x, y): ("); LCD.Write(endX); LCD.Write(", "); LCD.Write(endY); LCD.WriteLine(")");
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());
        LCD.Write("Intended Heading: "); LCD.WriteLine(desiredHeading);
        SD.Printf("goToPoint: Current (x, y): (%f, %f)\r\n", RPS.X(), RPS.Y());
        SD.Printf("goToPoint: Intended (x, y): (%f, %f)\r\n", endX, endY);
        SD.Printf("goToPoint: Current heading: %f\r\n", RPS.Heading());
        SD.Printf("goToPoint: Intended heading: %f\r\n", desiredHeading);

        // Making angle adjustments if necessary
        if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 1)
        {
            SD.Printf("goToPoint: Robot's heading is at least minorly off (3+ Degrees).\r\n");

            // This may need tweaked for when we're going really fast, but past a certain point that's an issue with your tolerance being too high
            if (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > 2 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 30)
            {
                SD.Printf("goToPoint: Heading MAJORLY off. Stopping and re-turning.\r\n");
                SD.Printf("goToPoint: StartHeading = %f, endHeading = %f\r\n", RPS.Heading(), desiredHeading);

                leftMotor.Stop();
                rightMotor.Stop();

                turn(endX, endY);
            }

            // Checking which direction we need to turn to get back to the correct heading and acting accordingly
            else if (shouldTurnLeft(RPS.Heading(), desiredHeading))
            {
                // SD.Printf("goToPoint: Turning slightly left to autocorrect.\r\n");
                leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .6);
                rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
            }

            else
            {
                // SD.Printf("goToPoint: Turning slightly right to autocorrect.\r\n");
                leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
                rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .6);
            }
        }

        // Otherwise, angle is fine, so set the motors back to going straight
        else
        {
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
        }

        // Doing this frequently to make sure it always has valid backups
        updateLastValidRPSValues();

        // Waiting a little bit until we poll RPS again and check all our values to make sure the robot is positioning well
        Sleep(.001);

        // Doing this frequently to make sure it always has valid backups
        updateLastValidRPSValues();
    }

    // Stopping the motors outright
    leftMotor.Stop();
    rightMotor.Stop();

    if (shouldTurnToEndHeading)
        turn(endHeading);
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

void turnSouthAndGoUntilRPS(float startHeading)
{
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
}

/*
// I don't think we'll even end up using this, but I'm leaving it here regardless
void goToPointBackwards(float endX, float endY, float endHeading, bool shouldTurnToEndHeading, float distanceTolerance, float percentageOfFullSpeed, bool shouldHaveMaxTime, float maxTime)
{
    SD.Printf("goToPoint: Going to point (%f, %f) and End Heading %f.\r\n", endX, endY, endHeading);

    // Turn toward initial point, making sure we have accurate RPS data beforehand
    loopWhileNoRPS();
    turn180DegreesAway(endX, endY);

    clock_t startTime = clock();

    SD.Printf("goToPoint: Entering distance tolerance check.\r\n");
    while (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > distanceTolerance)
    {
        // If it has a max time, break out of this loop if it overdoes it
        if (shouldHaveMaxTime)
        {
            if (clock() - startTime > maxTime)
            {
                break;
            }
        }

        // Refreshing this value for the iteration
        float desiredHeading = rotate180Degrees(getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY));

        // Debug Output
        clearLCD();
        LCD.Write("Current (x, y): ("); LCD.Write(RPS.X()); LCD.Write(", "); LCD.Write(RPS.Y()); LCD.WriteLine(")");
        LCD.Write("Intended (x, y): ("); LCD.Write(endX); LCD.Write(", "); LCD.Write(endY); LCD.WriteLine(")");
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());
        LCD.Write("Intended Heading: "); LCD.WriteLine(desiredHeading);
        SD.Printf("backwardsGoToPoint: Current (x, y): (%f, %f)\r\n", RPS.X(), RPS.Y());
        SD.Printf("backwardsGoToPoint: Intended (x, y): (%f, %f)\r\n", endX, endY);
        SD.Printf("backwardsGoToPoint: Current heading: %f\r\n", RPS.Heading());
        SD.Printf("backwardsGoToPoint: Intended heading: %f\r\n", desiredHeading);

        if (goToPointRPSIsBad())
        {
            Sleep(.001);
            continue;
        }

        if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 1)
        {
            SD.Printf("goToPoint: Robot's heading is at least minorly off (3+ Degrees).\r\n");

            // If heading is seriously off and it's not super close to its intended destination (which would cause extreme fidgeting very close to the point, depending on the tolerance)
            if (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > 2 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 35)
            {
                SD.Printf("goToPoint: Heading MAJORLY off. Stopping and re-turning.\r\n");
                SD.Printf("goToPoint: StartHeading = %f, endHeading = %f\r\n", RPS.Heading(), desiredHeading);

                leftMotor.Stop();
                rightMotor.Stop();

                turn(endX, endY);
            }

            // Turning shouldn't necessarily be proportional to movement speed but it seems better because if we're going slower we want to be super accurate
            // Important - The motor values for these are switched when compared to the original shouldTurnLeft because this is going backwards
            else if (shouldTurnLeft(RPS.Heading(), desiredHeading))
            {
                SD.Printf("goToPoint: Turning slightly left to autocorrect.\r\n");
                leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
                rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .6);
            }

            else
            {
                SD.Printf("goToPoint: Turning slightly right to autocorrect.\r\n");
                leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .6);
                rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
            }
        }

        // Otherwise, just keep going straight because the degree tolerance is acceptible
        else
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
        }

        Sleep(.001);
    }

    // Stopping the motors outright
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(.5);

    if (shouldTurnToEndHeading)
        turn(endHeading);
}
*/

// Todo - Documentation
bool shouldTurnLeft(float startHeading, float endHeading)
{
    float cwDistance, ccwDistance;

    if (startHeading > endHeading) { cwDistance = startHeading - endHeading; } // CW No-Wrap
    else { cwDistance = (0 + startHeading) + (360 - endHeading); } // CW Wrap
    if (startHeading > endHeading) { ccwDistance = (0 + endHeading) + (360 - startHeading); } // CCW Wrap
    else { ccwDistance = endHeading - startHeading; } // CCW No-Wrap

    // If ccw is less than cw, return that it should turn left. Otherwise, return false
    return (ccwDistance <= cwDistance);
}

// Overloaded method that just calls original method but figures out the angle given two points
void turn (float endX, float endY) { turn(getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY)); }
void turn(float endHeading)
{
    SD.Printf("turn: Entered turn loop. Turning to %f\r\n", endHeading);

    // Returns faulty value if it goes over zero degrees
    while (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 4)
    {
        loopWhileNoRPS();

        // Debug
        clearLCD();
        LCD.WriteLine("Turning.");
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());
        LCD.Write("Intended Heading: "); LCD.WriteLine(endHeading);

        if (shouldTurnLeft(RPS.Heading(), endHeading))
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .6);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .6);
        }

        else
        {
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .6);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .6);
        }

        Sleep(.001);
    }

    leftMotor.Stop();
    rightMotor.Stop();
}

/*
void turn180DegreesAway (float endX, float endY) { turn180DegreesAway(getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY)); }
void turn180DegreesAway(float endHeading)
{
    SD.Printf("turn180DegreesAway: Entered turn away loop. Turning to %f\r\n", endHeading);

    // Returns faulty value if it goes over zero degrees
    while (smallestDistanceBetweenHeadings(RPS.Heading(), rotate180Degrees(endHeading)) > 7)
    {
        loopWhileNoRPS();

        // Debug
        clearLCD();
        LCD.WriteLine("Turning 180 Degrees Off.");
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());
        LCD.Write("Intended Heading: "); LCD.WriteLine(rotate180Degrees(endHeading));

        if (shouldTurnLeft(RPS.Heading(), rotate180Degrees(endHeading)))
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .6);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .6);
        }

        else
        {
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .6);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .6);
        }

        Sleep(.001);

        // Keep going with the current motor setting until RPS resolves to a legitimate value
        while (goToPointRPSIsBad())
        {
            Sleep(.001);
        }
    }

    // The turn is done, so stop both motors
    leftMotor.Stop();
    rightMotor.Stop();
}

// Does exactly what you think it does
float rotate180Degrees(float degrees)
{
    // Taking modulus by 360 accounts for the fact that 380 Degrees = 20 Degrees (not specifically that, but that general kind of thing)
    // fmod is just modulus but for two float values, which we need here
    return fmod((degrees + 180), 360.0);
}
*/

#endif
