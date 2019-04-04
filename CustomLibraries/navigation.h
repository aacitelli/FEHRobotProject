#ifndef CUSTOMNAVIGATION_H
#define CUSTOMNAVIGATION_H

// Requisite Libraries
// Custom Libraries
// Todo - Trim these down to what we actually need

#include <FEHLCD.h>
#include <FEHSD.h>
#include <FEHRPS.h>

#include "rps.h"
#include "utility.h"

void getBackToRPSFromDeadzone();
void turn(float endHeading);
void turn(float endX, float endY);

void turnNoRPS(float currentHeading, float endHeading);

using namespace std;

#define GOTOPOINT_COUNTS_PER_SECOND 10

/**
 * Oh boy, is this a fun method...
 *
 * @brief goToPoint, long story short, takes in an (x, y) coordinate and makes its way there, intelligently autocorrecting so that it automatically gets to within a very small tolerance of that point.
 * @param endX is the x-coordinate of the point you want the robot to go to.
 * @param endY is the y-coordinate of the point you want the robot to go to.
 * @param shouldTurnToEndHeading is a boolean representing whether or not the robot should turn to a passed-in heading all the way at the end. Mostly used for task positioning.
 * @param endHeading is the endHeading mentioned from the last parameter.
 * @param isTimed is a boolean representing whether or not the robot should exit this function after a specified amount of time has passed.
 * @param time is the time amount at which the function should be stopped if isTimed be true
 * @param shouldGoBackwards is a boolean representing whether or not the robot should go forwards during the travel phase or backwards. We only go backwards during foosball; Everywhere else is forwards.
 */
void goToPoint(float endX, float endY, bool shouldTurnToEndHeading, float endHeading, bool isTimed, float time, bool shouldGoBackwards, bool shouldGoFast)
{
    SD.Printf("----------------------------------\r\n");
    SD.Printf("goToPoint: Entered goToPoint.\r\n");
    SD.Printf("goToPoint: goToPoint Passed-In Parameters: \r\n");
    SD.Printf("goToPoint: End (x, y): (%f, %f)\r\n", endX, endY);
    SD.Printf("goToPoint: Should Turn To End Heading (1 = Yes, 0 = No): %d\r\n", shouldTurnToEndHeading);
    SD.Printf("goToPoint: endHeading: %f\r\n", endHeading);
    SD.Printf("goToPoint: Is Timed (1 = Yes, 0 = No): %d\r\n", isTimed);
    SD.Printf("goToPoint: Time: %f\r\n", time);
    SD.Printf("goToPoint: Should Go Backwards (1 = Yes, 0 = No): %d\r\n", shouldGoBackwards);

    // Ensures we go into turn() with valid RPS, and handles the case where we hit a deadzone during the RPS checks 
    // This is one of the weirder loops in the program, don't worry about how it works
    if (loopUntilValidRPS() == -2)
    {
        SD.Printf("goToPoint: Deadzone has become enabled again.\r\n");

        // Causes the program to skip certain goToPoint calls
        hasExhaustedDeadzone = true;

        // Does what you think it does
        SD.Printf("goToPoint: Turning roughly south and going until RPS.\r\n");
        getBackToRPSFromDeadzone();

        // Escapes this call of goToPoint because it doesn't really have RPS any more
        return;
    }

    SD.Printf("goToPoint: Entering initial alignment turn() function.\r\n");

    // If it's supposed to turn forwards, just turn towards the point 
    if (!shouldGoBackwards)
        turn(endX, endY);
    
    // Otherwise, if it's supposed to turn backwards, turn to 180 degrees away from that point 
    // Todo - Consider replacing RPS.X with lastValidX (and so on) throughout the code to always use the cached values (I don't think this is necessary, but could be a good sanity check)
    else 
        turn(rotate180Degrees(getDesiredHeading(RPS.X(), RPS.Y(), endX, endY))); // Todo - This statement could be cleaned up a bit logically 

    // Debug 
    SD.Printf("goToPoint: Entering distance tolerance check.\r\n");
    
    // Tolerance Loop Setup 
    int iterationCount  = 0;
    float desiredHeading;

    // Ensures we go into turn() with valid RPS, and handles the case where we hit a deadzone during the RPS checks
    // This is one of the weirder loops in the program, don't worry about how it works
    if (loopUntilValidRPS() == -2)
    {
        SD.Printf("goToPoint: Deadzone has become enabled again.\r\n");

        // Causes the program to skip certain goToPoint calls
        hasExhaustedDeadzone = true;

        // Does what you think it does
        SD.Printf("goToPoint: Turning roughly south and going until RPS.\r\n");
        getBackToRPSFromDeadzone();

        // Escapes this call of goToPoint because it doesn't really have RPS any more
        return;
    }

    // Step #2 of Method - Go To The Point 
    // This tolerance used to be a passed-in value, but I rewrote the whole thing to trust that it'll always get within this tolerance basically
    // Todo - Figure out what this tolerance should be in order for the robot to end up at the most precise point it can
    float currentOverallMotorPower = .2; // Used to link turn speeds to forward speed
    float tolerance = .75;
    if (shouldGoFast)
        tolerance = 1.5;
    while (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > tolerance)
    {
        // We're guaranteed to have good RPS here
        updateLastValidRPSValues();

        // Timing check (this is basically the Proteus version of a timer using Sleep() method calls)
        if (isTimed)
        {
            SD.Printf("goToPoint: Current Iteration Count: %f\r\n", iterationCount);
            SD.Printf("goToPoint: Max Iteration Count: %f\r\n", (time * GOTOPOINT_COUNTS_PER_SECOND));

            iterationCount++;
            if (iterationCount > (time * GOTOPOINT_COUNTS_PER_SECOND))
            {
                break;
            }
        }

        if (!shouldGoBackwards)
            desiredHeading = getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY);
        else
            desiredHeading = rotate180Degrees(getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY));

        // Debug Output
        SD.Printf("goToPoint: Current Position: (%f, %f).\r\n", RPS.X(), RPS.Y());
        SD.Printf("goToPoint: Intended Position: (%f, %f)\r\n", endX, endY);
        SD.Printf("goToPoint: Current Heading: %f\r\n", RPS.Heading());
        SD.Printf("goToPoint: Desired Heading: %f\r\n", desiredHeading);
        SD.Printf("currentOverallMotorPower this iteration of the tolerance loop: %f\r\n", currentOverallMotorPower);

        /* DECISIONS, DECISIONS, ALL OF THEM WRONG */
        // Needs to autocorrect angularly this cycle
        if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 3)
        {
            // Can't feasibly correct in time, so it stops and turns
            if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 30)
            {
                SD.Printf("goToPoint: Heading MAJORLY off. Stopping and re-turning.\r\n");

                leftMotor.Stop();
                currentLeftMotorPercent = 0;

                rightMotor.Stop();
                currentRightMotorPercent = 0;

                turn(endX, endY);
            }

            // Forwards Corrections
            if (!shouldGoBackwards)
            {
                // Left Correction
                if (shouldTurnLeft(RPS.Heading(), desiredHeading))
                {
                    // Small Correction Necessary
                    if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 0 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 15)
                    {
                        SD.Printf("goToPoint: Given currentHeading = %f, endHeading = %f, turning slow-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);

                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * currentOverallMotorPower * .5);
                        currentLeftMotorPercent = LEFT_MOTOR_PERCENT * currentOverallMotorPower * .5;

                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * currentOverallMotorPower);
                        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * currentOverallMotorPower;
                    }

                    // Large Correction Necessary
                    else
                    {
                        SD.Printf("goToPoint: Given currentHeading = %f, endHeading = %f, turning fast-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);

                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * currentOverallMotorPower * .3);
                        currentLeftMotorPercent = LEFT_MOTOR_PERCENT * currentOverallMotorPower * .3;

                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * currentOverallMotorPower);
                        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * currentOverallMotorPower;
                    }
                }

                // Right Correction
                else
                {
                    // Small Correction
                    if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 0 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 15)
                    {
                        SD.Printf("goToPoint: Given currentHeading = %f, endHeading = %f, turning slow-speed right to autocorrect.\r\n", RPS.Heading(), desiredHeading);

                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * currentOverallMotorPower);
                        currentLeftMotorPercent = LEFT_MOTOR_PERCENT * currentOverallMotorPower;

                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * currentOverallMotorPower * .5);
                        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * currentOverallMotorPower * .5;
                    }

                    // Large Correction
                    else
                    {
                        SD.Printf("goToPoint: Given currentHeading = %f, endHeading = %f, turning fast-speed right to autocorrect.\r\n", RPS.Heading(), desiredHeading);

                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * currentOverallMotorPower);
                        currentLeftMotorPercent = LEFT_MOTOR_PERCENT * currentOverallMotorPower;

                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * currentOverallMotorPower * .3);
                        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * currentOverallMotorPower * .3;
                    }
                }
            }

            // Backwards Corrections
            else
            {
                // Left Correction
                if (shouldTurnLeft(RPS.Heading(), desiredHeading))
                {
                    // Small Correction
                    if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 0 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 15)
                    {
                        SD.Printf("goToPoint: (Going backwards) Given currentHeading = %f, endHeading = %f, turning slow-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);

                        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * currentOverallMotorPower);
                        currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * currentOverallMotorPower;

                        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * currentOverallMotorPower * .5);
                        currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * currentOverallMotorPower * .5;
                    }

                    // Large Correction
                    else
                    {
                        SD.Printf("goToPoint: (Going backwards) Given currentHeading = %f, endHeading = %f, turning fast-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);

                        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * currentOverallMotorPower);
                        currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * currentOverallMotorPower;

                        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * currentOverallMotorPower * .3);
                        currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * currentOverallMotorPower * .3;
                    }
                }

                // Right Correction
                else
                {
                    // Small Correction
                    if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 0 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 15)
                    {
                        SD.Printf("goToPoint: (Going backwards) Given currentHeading = %f, endHeading = %f, turning slow-speed right to autocorrect.\r\n", RPS.Heading(), desiredHeading);

                        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * currentOverallMotorPower * .5);
                        currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * currentOverallMotorPower * .5;

                        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * currentOverallMotorPower);
                        currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * currentOverallMotorPower;
                    }

                    // Large Correction
                    else
                    {
                        SD.Printf("goToPoint: (Going backwards) Given currentHeading = %f, endHeading = %f, turning fast-speed right to autocorrect.\r\n", RPS.Heading(), desiredHeading);

                        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * currentOverallMotorPower * .3);
                        currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * currentOverallMotorPower * .3;

                        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * currentOverallMotorPower);
                        currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * currentOverallMotorPower;
                    }
                }
            }
        }

        // Otherwise, it can just go forward this cycle
        else
        {
            // Forwards, No Turn
            if (!shouldGoBackwards)
            {
                // This is basically a special case - All instances where we have timed loops are where we want slow speeds (this change is here for DDR)
                if (isTimed)
                {
                    leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .2);
                    currentLeftMotorPercent = .2;

                    rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .2);
                    currentRightMotorPercent = .2;

                    currentOverallMotorPower = .2;
                }

                // Long distance, fast speed
                else if (getDistance(RPS.X(), RPS.Y(), endX, endY) > 4)
                {                    
                    SD.Printf("goToPoint: Robot is in line with desired angle, and is 4+ inches away. Going straight at full speed.\r\n");

                    if (shouldGoFast)
                    {
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .7);
                        currentLeftMotorPercent = LEFT_MOTOR_PERCENT * .7;

                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .7);
                        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .7;

                        currentOverallMotorPower = .7;
                    }

                    else
                    {
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .4);
                        currentLeftMotorPercent = LEFT_MOTOR_PERCENT * .4;

                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .4);
                        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .4;

                        currentOverallMotorPower = .4;
                    }
                }

                // Close distance, low speed
                else
                {
                    if (shouldGoFast)
                    {
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .4);
                        currentLeftMotorPercent = LEFT_MOTOR_PERCENT * .4;

                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .4);
                        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .4;

                        currentOverallMotorPower = .4;
                    }

                    else
                    {
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .2);
                        currentLeftMotorPercent = LEFT_MOTOR_PERCENT * .2;

                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .2);
                        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .2;

                        currentOverallMotorPower = .2;
                    }
                }
            }

            // Backwards, No Turn
            else
            {
                // Medium distance, medium speed
                if (getDistance(RPS.X(), RPS.Y(), endX, endY) > 4)
                {
                    SD.Printf("goToPoint: Robot is in line with desired angle, and is 2.5-4 inches away. Going straight at slightly decreased speed.\r\n");

                    if (shouldGoFast)
                    {
                        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .7);
                        currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * .7;

                        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .7);
                        currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * .7;

                        currentOverallMotorPower = .7;
                    }

                    else
                    {
                        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .4);
                        currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * .4;

                        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .4);
                        currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * .4;

                        currentOverallMotorPower = .4;
                    }
                }

                // Close distance, low speed
                else
                {
                    SD.Printf("goToPoint: Robot is in line with desired angle, but is closer than 2.5 inches away. Going straight slowly.\r\n");

                    if (shouldGoFast)
                    {
                        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .4);
                        currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * .4;

                        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .4);
                        currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * .4;

                        currentOverallMotorPower = .4;
                    }

                    else
                    {
                        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .2);
                        currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * .2;

                        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .2);
                        currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * .2;

                        currentOverallMotorPower = .2;
                    }
                }
            }
        }

        // Post-Logic Debug
        SD.Printf("goToPoint: Finalized motor powers at end of tolerance loop: \r\n");
        SD.Printf("goToPoint: Left Motor: %f\r\n", currentLeftMotorPercent);
        SD.Printf("goToPoint: Right Motor: %f\r\n", currentRightMotorPercent);

        // Letting a little bit of time elapse before we test new stuff
        Sleep(.025);

        // Ensures fresh RPS for next tolerance check and handles deadzone behavior
        if (loopUntilValidRPS() == -2)
        {
            // Debug
            SD.Printf("goToPoint: Turning roughly south and going until RPS.\r\n");

            // Does exactly what you think it does
            getBackToRPSFromDeadzone();

            // Program will now skip the rest of the goToPoint calls up top
            hasExhaustedDeadzone = true;

            // Escapes this call of goToPoint because it doesn't really have RPS any more
            return;
        }
    }

    SD.Printf("goToPoint: goToPoint is done; Stopping motors.\r\n");

    // Stopping the motors outright
    leftMotor.Stop();
    currentLeftMotorPercent = 0;

    rightMotor.Stop();
    currentRightMotorPercent = 0;

    // Step 3 Of Method - Turn to End Heading 
    if (shouldTurnToEndHeading)
    {
        SD.Printf("goToPoint: shouldTurnToEndHeading is true, so calling turn() with %f\r\n", endHeading);
        turn(endHeading);
    }

    // Crude benchmark debug system
    SD.Printf("///////////////////////////////\r\n");
    SD.Printf("goToPoint: FUNCTION SYNOPSIS: \r\n");
    SD.Printf("goToPoint: Intended (x, y): (%f, %f)\r\n", endX, endY);
    SD.Printf("goToPoint: Actual (x, y) @ End: (%f, %f)\r\n", RPS.X(), RPS.Y());

    if (shouldTurnToEndHeading)
    {
        SD.Printf("goToPoint: Intended Heading: %f\r\n", endHeading);
        SD.Printf("goToPoint: Actual Heading @ End: %f\r\n", RPS.Heading());
    }

    else
    {
        SD.Printf("Function was not instructed to turn to an end heading.\r\n");
    }

    SD.Printf("///////////////////////////////\r\n");
}

// Todo - Split the deadzone into small sectors, each of which has its own pathfinding to get back to RPS w/o hitting the dodecahedron 
// Todo (contd.) - The code keeps track of the last valid RPS values, so use that X, Y, and Heading to get back to south
// Todo - Test this (If this already works correctly, it'll be a literal miracle )
void getBackToRPSFromDeadzone()
{
    // Todo - Set up two different paths (to the left of dodecahedron, and to the right) for the robot to go 

    // If it's definitely to the left or the right of the dodecahedron (meaning we can go straight down in either case) 
    // Todo - Use rpsTest to figure out the actual values for these, then give it like 3 inches of leeway on each side - These are educated guesses rn 
    // (This isn't worth adding calibration for unless we're consistently having issues with it.
    // In a perfect run, none of these deadzone failsafes even trigger, but they're here in the case that they need to do something 
    if (lastValidX < 7 || lastValidX > 24)
    {
        turnNoRPS(lastValidHeading, 270);
    }

    // If it's somewhere generally above the dodecahedron (meaning we should go east for a bit then go straight down)
    // Todo - Make the amount we go to the side scale with how close we were to the dodecahedron's center x-coordinate 
    else 
    {
        // Turning as close to east as we can get
        turnNoRPS(lastValidHeading, 0);
        
        // Going that way for about a second 
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .5);
        currentLeftMotorPercent = LEFT_MOTOR_PERCENT * .5;

        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .5);
        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .5;

        Sleep(.5);

        // Stopping the motors
        leftMotor.Stop();
        currentLeftMotorPercent = 0;

        rightMotor.Stop();
        currentRightMotorPercent = 0;

        // Turning as close to south as we can get 
        // Makes the assumption that we're currently faced towards zero degrees
        turnNoRPS(0, 270);
    }

    // Drives until we get RPS back (used for all escape cases)
    leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .4);
    currentLeftMotorPercent = LEFT_MOTOR_PERCENT * .4;

    rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .4);
    currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .4;

    while (RPS.X() == -1 || RPS.X() == -2) { Sleep(.01); }

    // Waits another second once we get RPS to make sure we're firmly in RPS territory
    Sleep(1.0);

    leftMotor.Stop();
    currentLeftMotorPercent = 0;

    rightMotor.Stop();
    currentRightMotorPercent = 0;
}

// Overloaded method that takes in an (x, y) coordinate instead of a heading
// This will get you to the angle +- roughly 15 degrees - I'm working on trying to make that more reliable, though I don't want to resort to super slow turning near the end, because don't need super perfect precision (goToPoint autocorrects)
void turn (float endX, float endY) { turn(getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY)); }
void turn (float endHeading)
{    
    SD.Printf("turn: Entered function with currentHeading %f and endHeading %f.\r\n", RPS.Heading(), endHeading);

    // Don't want to check the tolerance check until RPS is completely valid 
    // Todo - Replace this w/ the more exhaustive check
    if (loopUntilValidRPS() == -2)
    {
        SD.Printf("goToPoint: Deadzone has become enabled again.\r\n");

        // Causes the program to skip certain goToPoint calls
        hasExhaustedDeadzone = true;

        // Does what you think it does
        SD.Printf("goToPoint: Turning roughly south and going until RPS.\r\n");
        getBackToRPSFromDeadzone();

        // Escapes this call of goToPoint because it doesn't really have RPS any more
        return;
    }

    // Todo - Make it start turning even if it doesn't have RPS based on last remembered values so that we don't have to wait for RPS to be valid to start 
    // Generally, turn() is called as part of goToPoint, which can easily make small autocorrections, hence why this threshold doesn't need to be super small   
    while (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 8)
    {
        clearLCD();
        LCD.Write("Current Heading: "); LCD.WriteLine(RPS.Heading());
        LCD.Write("Intended Heading: "); LCD.WriteLine(endHeading);

        // * At this point, RPS values are always valid and completely up to date due to the update at the end of the tolerance loop 
        // Because RPS is guaranteed to be valid right now, update the RPS cache 
        updateLastValidRPSValues();

        loopUntilValidRPS();

        // Debug 
        SD.Printf("turn: Given currentHeading = %f and endHeading = %f, going through another iteration of the tolerance loop.\r\n", RPS.Heading(), endHeading);

        // If turning left is quicker 
        if (shouldTurnLeft(RPS.Heading(), endHeading))
        {
            SD.Printf("turn: Given currentHeading = %f and endHeading = %f, robot is turning left.\r\n", RPS.Heading(), endHeading);

            // Todo - If optimizing for time, see how low we can get these thresholds while still being precise enough when it matters 
            // 50+ Degrees Away - Turn as quickly as possible
            if (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 50)
            {
                SD.Printf("turn: Robot is more than 60 degrees away from endHeading. Turning really fast.\r\n");

                leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .4);
                currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * .4;

                rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .5);
                currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .5;
            }

            // 25-50 Degrees Away - Turn quick, but not super quick
            else if (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 25)
            {
                SD.Printf("turn: Robot is more than 30 degrees away from endHeading. Turning fast, but not super fast.\r\n");

                leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .4);
                currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * .4;

                rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .4);
                currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .4;
            }

            // 0-25 Degrees Away - Turn slowly (precision matters)
            else
            {
                SD.Printf("turn: Robot is less than 30 degrees away from endHeading. Turning more slowly.\r\n");

                leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .2);
                currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * .2;

                rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .2);
                currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .2;
            }
        }

        // Otherwise, turning right is quicker 
        else
        {
            SD.Printf("turn: Given currentHeading = %f and endHeading = %f, robot is turning right.\r\n", RPS.Heading(), endHeading);

            // 50+ Degrees Away - Turn as quickly as possible
            if (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 50)
            {
                SD.Printf("turn: Robot is more than 30 degrees away from endHeading. Turning faster.\r\n");

                leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .5);
                currentLeftMotorPercent = LEFT_MOTOR_PERCENT * .5;

                rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .5);
                currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * .5;
            }

            // 25-50 Degrees Away - Turn quick, but not super quick
            else if (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 25)
            {
                SD.Printf("turn: Robot is more than 30 degrees away from endHeading. Turning faster.\r\n");

                leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .4);
                currentLeftMotorPercent = LEFT_MOTOR_PERCENT * .4;

                rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .4);
                currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * .4;
            }

            // 0-25 Degrees Away - Turn slowly (precision matters)
            else 
            {
                SD.Printf("turn: Robot is less than 30 degrees away from endHeading. Turning more slowly.\r\n");

                leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .2);
                currentLeftMotorPercent = LEFT_MOTOR_PERCENT * .2;

                rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .2);
                currentRightMotorPercent = -RIGHT_MOTOR_PERCENT * .2;
            }
        }

        SD.Printf("turn: Finalized motor powers at end of tolerance loop: \r\n");
        SD.Printf("turn: Left Motor: %f\r\n", currentLeftMotorPercent);
        SD.Printf("turn: Right Motor: %f\r\n", currentRightMotorPercent);

        Sleep(.01);

        // This function itself is naturally blocking; The return value is only relevant if it's in a deadzone 
        if (loopUntilValidRPS() == -2)
        {
            SD.Printf("goToPoint: Deadzone has become enabled again.\r\n");

            // Causes the program to skip certain goToPoint calls
            hasExhaustedDeadzone = true;

            // Does what you think it does
            SD.Printf("goToPoint: Turning roughly south and going until RPS.\r\n");
            getBackToRPSFromDeadzone();

            // Escapes this call of goToPoint because it doesn't really have RPS any more
            return;
        }
    }

    leftMotor.Stop();
    currentLeftMotorPercent = 0;

    rightMotor.Stop();
    currentRightMotorPercent = 0;

    // Crude benchmark debug system
    SD.Printf("///////////////////////////////\r\n");
    SD.Printf("turn: FUNCTION SYNOPSIS: \r\n");
    SD.Printf("turn: Intended Heading: %f\r\n", endHeading);
    SD.Printf("turn: Actual Heading @ End: %f\r\n", RPS.Heading());
    SD.Printf("///////////////////////////////\r\n");
}

void turnToAngleWhenAlreadyReallyClose(float endHeading)
{
    // This function itself is naturally blocking; The return value is only relevant if it's in a deadzone
    if (loopUntilValidRPS() == -2)
    {
        SD.Printf("goToPoint: Deadzone has become enabled again.\r\n");

        // Causes the program to skip certain goToPoint calls
        hasExhaustedDeadzone = true;

        // Does what you think it does
        SD.Printf("goToPoint: Turning roughly south and going until RPS.\r\n");
        getBackToRPSFromDeadzone();

        // Escapes this call of goToPoint because it doesn't really have RPS any more
        return;
    }

    while (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 1.5)
    {
        if (shouldTurnLeft(RPS.Heading(), endHeading))
        {
            leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .2);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .2);
        }

        else
        {
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .2);
            rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .2);
        }

        Sleep(.05);
        leftMotor.Stop();
        rightMotor.Stop();

        Sleep(.35);

        // This function itself is naturally blocking; The return value is only relevant if it's in a deadzone
        if (loopUntilValidRPS() == -2)
        {
            SD.Printf("goToPoint: Deadzone has become enabled again.\r\n");

            // Causes the program to skip certain goToPoint calls
            hasExhaustedDeadzone = true;

            // Does what you think it does
            SD.Printf("goToPoint: Turning roughly south and going until RPS.\r\n");
            getBackToRPSFromDeadzone();

            // Escapes this call of goToPoint because it doesn't really have RPS any more
            return;
        }
    }

    SD.Printf("///////////////////////////////\r\n");
    SD.Printf("accurateTurn: FUNCTION SYNOPSIS: \r\n");
    SD.Printf("accurateTurn: Intended Heading: %f\r\n", endHeading);
    SD.Printf("accurateTurn: Actual Heading @ End: %f\r\n", RPS.Heading());
    SD.Printf("///////////////////////////////\r\n");
}

// Like the normal turn method, but works based off of a last saved heading and an intended heading 
void turnNoRPS(float currentHeading, float endHeading)
{
    // Todo - See if I can up the motor speeds here a little bit to save time (only worth doing if we're at 100 points consistently) 
    // To up the motor speeds, I'd have to figure out Seconds_Per_Degree for every motor speed I want to use and implement them all...
    // That isn't really bad, but also isn't something I want to implement before we have more of the course consistently done 

    if (shouldTurnLeft(currentHeading, endHeading))
    {
        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .4);
        currentLeftMotorPercent = -LEFT_MOTOR_PERCENT * .4;

        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .4);
        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .4;
    }

    else 
    {
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .4);
        currentLeftMotorPercent = LEFT_MOTOR_PERCENT * .4;

        rightMotor.SetPercent(-LEFT_MOTOR_PERCENT * .4);
        currentRightMotorPercent = RIGHT_MOTOR_PERCENT * .4;
    }

    // Degrees * (Seconds / Degrees) = Seconds
    Sleep(smallestDistanceBetweenHeadings(currentHeading, endHeading) * SECONDS_PER_DEGREE);
}

#endif
