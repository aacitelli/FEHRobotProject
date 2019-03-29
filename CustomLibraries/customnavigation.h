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
float rpsXToCentroidX();
float rpsYToCentroidY();

using namespace std;

#define GOTOPOINT_COUNTS_PER_SECOND 10

// Todo - Make a version of goToPoint that goes backwards (instead of trying to optimize for whether backwards is faster or not algorithmically - Hardcode whether it's forwards or backwards)

void goToPoint(float endX, float endY, float distanceTolerance, float percentageOfFullSpeed, bool shouldTurnToEndHeading, float endHeading, bool isTimed, float time)
{
    SD.Printf("goToPoint: Going to point (%f, %f) and End Heading %f.\r\n", endX, endY, endHeading);

    // Turn toward initial point, making sure we have accurate RPS data beforehand
    loopWhileNoRPS();
    turn(endX, endY);

    SD.Printf("goToPoint: Entering distance tolerance check.\r\n");
    int iterationCount  = 0;
    while (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > distanceTolerance)
    {
        // Handling course stuff
        if (RPSIsBad())
        {
            SD.Printf("Current goToPoint tolerance loop has invalid RPS.\r\n");
            // Check for deadzone and go south if it ever detects it, triggering something that makes it skip relevant goToPoint methods if it goes over time
            if (RPS.X() == -2 || RPS.Y() == -2 || RPS.Heading() == -2)
            {
                SD.Printf("Deadzone has become enabled again.\r\n");

                // Causes the program to skip certain goToPoint calls
                hasExhaustedDeadzone = true;

                // Does what you think it does
                SD.Printf("Turning roughly south and going until RPS.\r\n");
                turnSouthAndGoUntilRPS(lastValidHeading);

                // Escapes this call of goToPoint because it doesn't really have RPS any more
                return;
            }

            Sleep(.01);
            continue;
        }

        // If this instance of goToPoint is timed, check for that
        if (isTimed)
        {
            SD.Printf("Current Iteration Count: %f\r\n", iterationCount);
            SD.Printf("Max Iteration Count: %f\r\n", (time * GOTOPOINT_COUNTS_PER_SECOND));

            iterationCount++;
            if (iterationCount > (time * GOTOPOINT_COUNTS_PER_SECOND))
            {
                break;
            }
        }

        // If it gets to this point, RPS values are valid, meaning we can update everything and make new, updated decisions for positioning
        updateLastValidRPSValues();
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

        // Making angle adjustments if necessary
        if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 5)
        {
            SD.Printf("goToPoint: Robot's heading is at least minorly off (3+ Degrees).\r\n");

            // This may need tweaked for when we're going really fast, but past a certain point that's an issue with your tolerance being too high
            if (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > 2 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 20)
            {
                SD.Printf("goToPoint: Heading MAJORLY off. Stopping and re-turning.\r\n");

                leftMotor.Stop();
                rightMotor.Stop();

                turn(endX, endY);
            }

            // Checking which direction we need to turn to get back to the correct heading and acting accordingly
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

        // Otherwise, angle is fine, so set the motors back to going straight
        else
        {
            SD.Printf("Robot is in line with desired angle. Going straight at full speed.\r\n");
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
        }

        // Waiting a little bit until we poll RPS again and check all our values to make sure the robot is positioning well
        Sleep(.001);
    }

    // Stopping the motors outright
    SD.Printf("goToPoint is done; Stopping motors.\r\n");

    leftMotor.Stop();
    rightMotor.Stop();

    // Give time for the robot to stop before we start turning 
    Sleep(.5);

    if (shouldTurnToEndHeading)
    {
        SD.Printf("shouldTurnToEndHeading is true, so turning to %f\r\n", endHeading);
        turn(endHeading);
    }
}

// Actually turns to 290 to hopefully avoid the dodecahedron
void turnSouthAndGoUntilRPS(float startHeading)
{
    // 0 to 90
    if (startHeading >= 0 && startHeading < 160)
    {
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT);
        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT);

        Sleep(SECONDS_PER_DEGREE * (startHeading + 70));
    }

    // 90 to 270
    else if (startHeading >= 90 && startHeading < 290)
    {
        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT);

        Sleep(SECONDS_PER_DEGREE * (290 - startHeading));
    }

    // 270 to 360
    else
    {
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT);
        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT);

        Sleep(SECONDS_PER_DEGREE * (360 - startHeading));
    }

    leftMotor.SetPercent(LEFT_MOTOR_PERCENT);
    rightMotor.SetPercent(RIGHT_MOTOR_PERCENT);

    while (RPS.X() == -1 || RPS.X() == -2) { Sleep(.01); }
    Sleep(1.0);

    leftMotor.Stop();
    rightMotor.Stop();
}

// I don't think we'll even end up using this, but I'm leaving it here regardless
void goToPointBackwards(float endX, float endY, float distanceTolerance, float percentageOfFullSpeed, bool shouldTurnToEndHeading, float endHeading, bool isTimed, float time)
{
    SD.Printf("goToPoint: Going to point (%f, %f) and End Heading %f.\r\n", endX, endY, endHeading);

    // Turn toward initial point, making sure we have accurate RPS data beforehand
    loopWhileNoRPS();
    turn180DegreesAway(endX, endY);

    SD.Printf("goToPoint: Entering distance tolerance check.\r\n");
    int iterationCount  = 0;
    while (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > distanceTolerance)
    {
        // Handling course stuff
        if (RPSIsBad())
        {
            // Check for deadzone and go south if it ever detects it, triggering something that makes it skip relevant goToPoint methods if it goes over time
            if (RPS.X() == -2 || RPS.Y() == -2 || RPS.Heading() == -2)
            {
                // Causes the program to skip certain goToPoint calls
                hasExhaustedDeadzone = true;

                // Does what you think it does
                turnSouthAndGoUntilRPS(lastValidHeading);
                return; // Escapes the function
            }

            Sleep(.01);
            continue;
        }

        // If this instance of goToPoint is timed, check for that
        if (isTimed)
        {
            iterationCount++;
            if (iterationCount > (time * GOTOPOINT_COUNTS_PER_SECOND))
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
                leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .625);
                rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .75);
            }

            else
            {
                SD.Printf("goToPoint: Turning slightly right to autocorrect.\r\n");
                leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .75);
                rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .625);
            }
        }

        // Otherwise, just keep going straight because the degree tolerance is acceptible
        else
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
        }

        Sleep(.01);
    }

    // Stopping the motors outright
    leftMotor.Stop();
    rightMotor.Stop();

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
void turn (float endHeading)
{
    SD.Printf("turn: Entered turn loop. Turning to %f\r\n", endHeading);

    // Returns faulty value if it goes over zero degrees
    while (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 3)
    {
        loopWhileNoRPS();

        // Debug
        clearLCD();
        LCD.WriteLine("Turning.");
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());
        LCD.Write("Intended Heading: "); LCD.WriteLine(endHeading);

        if (shouldTurnLeft(RPS.Heading(), endHeading))
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .15);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .15);
        }

        else
        {
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .15);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .15);
        }

        Sleep(.01);
    }

    leftMotor.Stop();
    rightMotor.Stop();
}

void turn180DegreesAway (float endX, float endY) { turn180DegreesAway(getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY)); }
void turn180DegreesAway(float endHeading)
{
    SD.Printf("turn180DegreesAway: Entered turn away loop. Turning to %f\r\n", endHeading);

    // Returns faulty value if it goes over zero degrees
    while (smallestDistanceBetweenHeadings(RPS.Heading(), rotate180Degrees(endHeading)) > 5)
    {
        loopWhileNoRPS();

        // Debug
        clearLCD();
        LCD.WriteLine("Turning 180 Degrees Off.");
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());
        LCD.Write("Intended Heading: "); LCD.WriteLine(rotate180Degrees(endHeading));

        if (shouldTurnLeft(RPS.Heading(), rotate180Degrees(endHeading)))
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .5);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .5);
        }

        else
        {
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .5);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .5);
        }

        Sleep(.001);
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

// This isn't used, but will be used if our QR code dramatically changes position
float rpsXToCentroidX()
{
    float returnValue;

    // 0 Degrees (Inclusive) to 90 Degrees (Exclusive)
    if (RPS.Heading() >= 0 && RPS.Heading() < 90)
        returnValue = RPS.X() + DISTANCE_BETWEEN_RPS_AND_CENTROID * cos(degreeToRadian(RPS.Heading()));

    // 90 Degrees (Inclusive) to 180 Degrees (Exclusive)
    else if (RPS.Heading() >= 90 && RPS.Heading() < 180)
        returnValue = RPS.X() - (DISTANCE_BETWEEN_RPS_AND_CENTROID * sin(degreeToRadian(RPS.Heading() - 90)));

    // 180 Degrees (Inclusve) to 270 Degrees (Exclusive)
    else if (RPS.Heading() >= 180 && RPS.Heading() < 270)
        returnValue = RPS.X() - (DISTANCE_BETWEEN_RPS_AND_CENTROID * cos(degreeToRadian(RPS.Heading() - 180)));

    // 270 Degrees (Inclusive) to 360 Degrees (Inclusive)
    else
        returnValue = RPS.X() + (DISTANCE_BETWEEN_RPS_AND_CENTROID * sin(degreeToRadian(RPS.Heading() - 270)));

    return returnValue;
}

// Also isn't used, but will be used if our QR code dramatically changes position
float rpsYToCentroidY()
{
    float returnValue;

    // 0 Degrees (Inclusive) to 90 Degrees (Exclusive)
    if (RPS.Heading() >= 0 && RPS.Heading() < 90)
        returnValue = RPS.Y() + (DISTANCE_BETWEEN_RPS_AND_CENTROID * sin(degreeToRadian(RPS.Heading())));

    // 90 Degrees (Inclusive) to 180 Degrees (Exclusive)
    else if (RPS.Heading() >= 90 && RPS.Heading() < 180)
        returnValue = RPS.Y() + (DISTANCE_BETWEEN_RPS_AND_CENTROID * cos(degreeToRadian(RPS.Heading() - 90)));

    // 180 Degrees (Inclusive) to 270 Degrees (Exclusive)
    else if (RPS.Heading() >= 180 && RPS.Heading() < 270)
        returnValue = RPS.Y() - (DISTANCE_BETWEEN_RPS_AND_CENTROID * sin(degreeToRadian(RPS.Heading() - 180)));

    // 270 Degrees (Inclusive) to 360 Degrees (Inclusive)
    else
        returnValue = RPS.Y() - (DISTANCE_BETWEEN_RPS_AND_CENTROID * cos(degreeToRadian(RPS.Heading() - 270)));

    return returnValue;
}

#endif
