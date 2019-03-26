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
 * Possible Ways to Increase Relability:
 *
 * Implement shaft encoders so that we can dynamically alter wheel speed if it detects that one is moving slower than another (ensures we are going straight)
 * Implement quicker pathfinding (this is hard, but involves algorithmically minimizing distance traveled (and therefore time) and potentially going backwards during the travel route)
 *
 * */

#ifndef CUSTOMNAVIGATION_H
#define CUSTOMNAVIGATION_H

// Requisite Libraries
#include "customconstants.h" // Used to get references to the motors
#include "customutility.h" // Used for unit conversions and widely applicable functions

#include <FEHLCD.h>
#include <FEHSD.h>
#include <FEHRPS.h>

// Function Predeclarations
void turn(float endHeading);
void turn(float endX, float endY);
float getDesiredHeading(float x1, float y1, float x2, float y2);
bool shouldTurnLeft(float startHeading, float endHeading);
void turn180DegreesAway(float endHeading);
void turn180DegreesAway(float endX, float endY);
float rotate180Degrees(float degrees);

using namespace std;

// Todo - Make a version of goToPoint that goes backwards (instead of trying to optimize for whether backwards is faster or not algorithmically - Hardcode whether it's forwards or backwards)
bool goToPointRPSIsBad() { return (RPS.X() == -1 || RPS.Y() == -1 || RPS.Heading() == -1 || RPS.X() == -2 || RPS.Y() == -2 || RPS.Heading() == -2); }
void goToPoint(float endX, float endY, float endHeading, bool shouldTurnToEndHeading, float distanceTolerance, float percentageOfFullSpeed)
{
    SD.Printf("goToPoint: Going to point (%f, %f) and End Heading %f.\r\n", endX, endY, endHeading);

    // Turn toward initial point, making sure we have accurate RPS data beforehand
    loopWhileNoRPS();
    turn(endX, endY);

    SD.Printf("goToPoint: Entering distance tolerance check.\r\n");
    while (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > distanceTolerance)
    {
        // Refreshing this value for the iteration
        float desiredHeading = getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY);

        // Debug Output
        clearLCD();
        LCD.Write("Current (x, y): ("); LCD.Write(RPS.X()); LCD.Write(", "); LCD.Write(RPS.Y()); LCD.WriteLine(")");
        LCD.Write("Intended (x, y): ("); LCD.Write(endX); LCD.Write(", "); LCD.Write(endY); LCD.WriteLine(")");
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());
        LCD.Write("Intended Heading: "); LCD.WriteLine(desiredHeading);
        SD.Printf("goToPoint: Current (x, y): (%f, %f)\r\n", RPS.X(), RPS.Y());
        SD.Printf("goToPoint: Intended (x, y): (%f, %f)\r\n", endX, endY);
        SD.Printf("goToPoint: Current heading: %f\r\n", RPS.Heading());
        SD.Printf("goToPoint: Intended heading: %f\r\n", desiredHeading);

        if (goToPointRPSIsBad())
        {
            Sleep(.001);
            continue;
        }

        if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 1)
        {
            SD.Printf("goToPoint: Robot's heading is at least minorly off (3+ Degrees).\r\n");

            if (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > 2 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 20)
            {
                SD.Printf("goToPoint: Heading MAJORLY off. Stopping and re-turning.\r\n");
                SD.Printf("goToPoint: StartHeading = %f, endHeading = %f\r\n", RPS.Heading(), desiredHeading);

                leftMotor.Stop();
                rightMotor.Stop();

                turn(endX, endY);
            }

            // Turning shouldn't necessarily be proportional to movement speed but it seems better because if we're going slower we want to be super accurate
            else if (shouldTurnLeft(RPS.Heading(), desiredHeading))
            {
                SD.Printf("goToPoint: Turning slightly left to autocorrect.\r\n");
                leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .6);
                rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
            }

            else
            {
                SD.Printf("goToPoint: Turning slightly right to autocorrect.\r\n");
                leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
                rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .6);
            }
        }

        else
        {
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
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
void goToPointBackwards(float endX, float endY, float endHeading, bool shouldTurnToEndHeading, float distanceTolerance, float percentageOfFullSpeed)
{
    SD.Printf("goToPoint: Going to point (%f, %f) and End Heading %f.\r\n", endX, endY, endHeading);

    // Turn toward initial point, making sure we have accurate RPS data beforehand
    loopWhileNoRPS();
    turn180DegreesAway(endX, endY);

    SD.Printf("goToPoint: Entering distance tolerance check.\r\n");
    while (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > distanceTolerance)
    {
        // Refreshing this value for the iteration
        float desiredHeading = rotate180Degrees(getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY));

        // Debug Output
        clearLCD();
        LCD.Write("Current (x, y): ("); LCD.Write(RPS.X()); LCD.Write(", "); LCD.Write(RPS.Y()); LCD.WriteLine(")");
        LCD.Write("Intended (x, y): ("); LCD.Write(endX); LCD.Write(", "); LCD.Write(endY); LCD.WriteLine(")");
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());
        LCD.Write("Intended Heading: "); LCD.WriteLine(desiredHeading);
        SD.Printf("goToPoint: Current (x, y): (%f, %f)\r\n", RPS.X(), RPS.Y());
        SD.Printf("goToPoint: Intended (x, y): (%f, %f)\r\n", endX, endY);
        SD.Printf("goToPoint: Current heading: %f\r\n", RPS.Heading());
        SD.Printf("goToPoint: Intended heading: %f\r\n", desiredHeading);

        if (goToPointRPSIsBad())
        {
            Sleep(.001);
            continue;
        }

        if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 1)
        {
            SD.Printf("goToPoint: Robot's heading is at least minorly off (3+ Degrees).\r\n");

            // If heading is seriously off and it's not super close to its intended destination (which would cause extreme fidgeting very close to the point, depending on the tolerance)
            if (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > 2 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 20)
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
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .45);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .45);
        }

        else
        {
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .45);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .45);
        }

        Sleep(.001);

        while (goToPointRPSIsBad())
        {
            Sleep(.001);
        }
    }

    leftMotor.Stop();
    rightMotor.Stop();
}

void turn180DegreesAway (float endX, float endY) { turn180DegreesAway(getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY)); }
void turn180DegreesAway(float endHeading)
{
    SD.Printf("turn180DegreesAway: Entered turn away loop. Turning to %f\r\n", endHeading);

    // Returns faulty value if it goes over zero degrees
    while (smallestDistanceBetweenHeadings(RPS.Heading(), rotate180Degrees(endHeading)) > 4)
    {
        loopWhileNoRPS();

        // Debug
        clearLCD();
        LCD.WriteLine("Turning 180 Degrees Off.");
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());
        LCD.Write("Intended Heading: "); LCD.WriteLine(rotate180Degrees(endHeading));

        if (shouldTurnLeft(RPS.Heading(), rotate180Degrees(endHeading)))
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .45);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .45);
        }

        else
        {
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .45);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .45);
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

#endif
