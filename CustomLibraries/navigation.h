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

void turnNoRPS(float currentHeading, float endHeading);

// Function Predeclarations
// Todo - Get rid of these
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
void getBackToRPSFromDeadzone(float startX, float startY, float startHeading);

using namespace std;

#define GOTOPOINT_COUNTS_PER_SECOND 10

void goToPoint(float endX, float endY, float distanceTolerance, float percentageOfFullSpeed, bool shouldTurnToEndHeading, float endHeading, bool isTimed, float time, bool shouldGoBackwards)
{
    SD.Printf("----------------------------------");
    SD.Printf("goToPoint: Entered goToPoint.\r\n");
    SD.Printf("goToPoint: goToPoint Passed-In Parameters: \r\n");
    SD.Printf("goToPoint: End (x, y): (%f, %f)\r\n", endX, endY);
    SD.Printf("goToPoint: Distance Tolerance: %f\r\n", distanceTolerance);
    SD.Printf("goToPoint: Percentage of Full Speed: %f\r\n", percentageOfFullSpeed);
    SD.Printf("goToPoint: Should Turn To End Heading (1 = Yes, 2 = No): %d\r\n", shouldTurnToEndHeading);
    SD.Printf("goToPoint: endHeading: %f\r\n", endHeading);
    SD.Printf("goToPoint: Is Timed (1 = Yes, 2 = No): %d\r\n", isTimed);
    SD.Printf("goToPoint: Time: %f\r\n", time);
    SD.Printf("goToPoint: Should Go Backwards (1 = Yes, 2 = No): %d\r\n", shouldGoBackwards);

    // Ensures we go into turn() with valid RPS, and handles the case where we hit a deadzone during the RPS checks 
    // This is one of the weirder loops in the program, don't worry about how it works
    if (loopUntilValidRPS() == -2)
    {
        SD.Printf("goToPoint: Deadzone has become enabled again.\r\n");

        // Causes the program to skip certain goToPoint calls
        hasExhaustedDeadzone = true;

        // Does what you think it does
        SD.Printf("goToPoint: Turning roughly south and going until RPS.\r\n");
        getBackToRPSFromDeadzone(lastValidX, lastValidY, lastValidHeading);

        // Escapes this call of goToPoint because it doesn't really have RPS any more
        return;
    }

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

    // Step #2 of Method - Go To The Point 
    while (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > distanceTolerance)
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

        // If it gets to this point, RPS values are valid, meaning we can update everything and make new, updated decisions for positioning
        updateLastValidRPSValues();

        if (!shouldGoBackwards)
            desiredHeading = getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY);
        else 
            desiredHeading = rotate180Degrees(getDesiredHeading(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY));

        // Debug Output
        clearLCD();
        SD.Printf("goToPoint: Going (%f, %f) to (%f, %f)\r\n", RPS.X(), RPS.Y(), endX, endY);
        SD.Printf("goToPoint: Rotating %f to %f: %f\r\n", RPS.Heading(), desiredHeading);

        // Checking angles and making necessary adjustments 
        if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) > 5)
        {
            SD.Printf("goToPoint: Robot's heading is at least minorly off (3+ Degrees).\r\n");

            // Todo - This is a LOT of code - Consider further methodizing this to make it more readable (though I kinda like it how it is)

            // This may need tweaked for when we're going really fast, but past a certain point that's an issue with your tolerance being too high
            if (getDistance(rpsXToCentroidX(), rpsYToCentroidY(), endX, endY) > 1 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 30)
            {
                SD.Printf("goToPoint: Heading MAJORLY off. Stopping and re-turning.\r\n");

                leftMotor.Stop();
                rightMotor.Stop();

                turn(endX, endY);
            }

            // Checking which direction we need to turn to get back to the correct heading and acting accordingly
            else if (shouldTurnLeft(RPS.Heading(), desiredHeading))
            {
                if (!shouldGoBackwards)
                {
                    // Small Correction Necessary (0-10 Degrees - Small disparity between motor powers )
                    if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 0 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 10)
                    {
                        SD.Printf("goToPoint: Given currentHeading = %f, endHeading = %f, turning slow-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .8);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
                    }

                    // Medium Correction Necessary (10-20 Degrees - Medium disparity between motor powers )
                    else if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 10 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 20)
                    {
                        SD.Printf("goToPoint: Given currentHeading = %f, endHeading = %f, turning medium-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .7);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
                    }

                    // Large Correction Necessary (20-30 Degrees - Largest disparity between motor powers)
                    else if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 20 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 30)
                    {
                        SD.Printf("goToPoint: Given currentHeading = %f, endHeading = %f, turning fast-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .6);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
                    }
                }  

                // If it's going backwards, the motor powers need to be inverted, otherwise they'll autocorrect the wrong way 
                else 
                {
                    // Small Correction Necessary (0-10 Degrees - Smallest disparity between motor powers)
                    if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 0 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 10)
                    {
                        SD.Printf("goToPoint: (Going backwards) Given currentHeading = %f, endHeading = %f, turning slow-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .8);
                    }

                    // Medium Correction Necessary (10-20 Degrees - Medium disparity between motor powers)
                    else if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 10 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 20)
                    {
                        SD.Printf("goToPoint: (Going backwards) Given currentHeading = %f, endHeading = %f, turning medium-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .7);
                    }

                    // Large Correction Necessary (20-30 Degrees - Largest disparity between motor powers)
                    else if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 20 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 30)
                    {
                        SD.Printf("goToPoint: (Going backwards) Given currentHeading = %f, endHeading = %f, turning fast-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .6);
                    }
                }              
            }

            // Otherwise, it needs to turn right 
            else
            {
                if (!shouldGoBackwards)
                {
                    // Small Correction Necessary (0-10 Degrees - Smallest disparity between motor powers)
                    if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 0 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 10)
                    {
                        SD.Printf("goToPoint: Given currentHeading = %f, endHeading = %f, turning slow-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .8);
                    }

                    // Medium Correction Necessary (10-20 Degrees - Medium disparity between motor powers)
                    else if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 10 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 20)
                    {
                        SD.Printf("goToPoint: Given currentHeading = %f, endHeading = %f, turning medium-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .7);
                    }

                    // Large Correction Necessary (20-30 Degrees - Largest disparity between motor powers)
                    else if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 20 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 30)
                    {
                        SD.Printf("goToPoint: Given currentHeading = %f, endHeading = %f, turning fast-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed * .6);
                    }
                }

                else 
                {
                    // Small Correction Necessary (0-10 Degrees - Small disparity between motor powers )
                    if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 0 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 10)
                    {
                        SD.Printf("goToPoint: (Going backwards) Given currentHeading = %f, endHeading = %f, turning slow-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .8);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
                    }

                    // Medium Correction Necessary (10-20 Degrees - Medium disparity between motor powers )
                    else if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 10 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 20)
                    {
                        SD.Printf("goToPoint: (Going backwards) Given currentHeading = %f, endHeading = %f, turning medium-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .7);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
                    }

                    // Large Correction Necessary (20-30 Degrees - Largest disparity between motor powers)
                    else if (smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) >= 20 && smallestDistanceBetweenHeadings(RPS.Heading(), desiredHeading) < 30)
                    {
                        SD.Printf("goToPoint: (Going backwards) Given currentHeading = %f, endHeading = %f, turning fast-speed left to autocorrect.\r\n", RPS.Heading(), desiredHeading);
                        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed * .6);
                        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
                    }
                }                
            }
        }

        // Otherwise, angle is fine, so make sure the motors are going straight 
        else
        {
            SD.Printf("goToPoint: Robot is in line with desired angle. Going straight at full speed.\r\n");
            leftMotor.SetPercent(LEFT_MOTOR_PERCENT * percentageOfFullSpeed);
            rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * percentageOfFullSpeed);
        }

        // Waiting a bit until last tolerance check 
        updateLastValidRPSValues();
        Sleep(.001);

        // Guarantees that the tolerance check always goes through with valid RPS, and also handles the deadzone check 
        if (loopUntilValidRPS() == -2)
        {
            SD.Printf("goToPoint: Deadzone has become enabled again.\r\n");

            // Causes the program to skip certain goToPoint calls
            hasExhaustedDeadzone = true;

            // Does what you think it does
            SD.Printf("goToPoint: Turning roughly south and going until RPS.\r\n");
            getBackToRPSFromDeadzone(lastValidX, lastValidY, lastValidHeading);

            // Escapes this call of goToPoint because it doesn't really have RPS any more
            return;
        }
    }

    // Stopping the motors outright
    SD.Printf("goToPoint: goToPoint is done; Stopping motors.\r\n");

    leftMotor.Stop();
    rightMotor.Stop();

    // Give time for the robot to stop before we start turning 
    Sleep(.5);

    // Step 3 Of Method - Turn to End Heading 
    if (shouldTurnToEndHeading)
    {
        SD.Printf("goToPoint: shouldTurnToEndHeading is true, so calling turn() with %f\r\n", endHeading);
        turn(endHeading);
    }
}

// Todo - Split the deadzone into small sectors, each of which has its own pathfinding to get back to RPS w/o hitting the dodecahedron 
// Todo (contd.) - The code keeps track of the last valid RPS values, so use that X, Y, and Heading to get back to south
// Todo - Test this (I seriously doubt this is fully functional) 
void getBackToRPSFromDeadzone(float startX, float startY, float startHeading)
{
    // Todo - Set up two different paths (to the left of dodecahedron, and to the right) for the robot to go 

    // If it's definitely to the left or the right of the dodecahedron (meaning we can go straight down in either case) 
    // Todo - Use rpsTest to figure out the actual values for these, then give it like 3 inches of leeway on each side - These are educated guesses rn 
    // (This isn't worth adding calibration for unless we're consistently having issues with it.
    // In a perfect run, none of these deadzone failsafes even trigger, but they're here in the case that they need to do something 
    if (lastValidX < 8 || lastValidX > 16)
    {
        turnNoRPS(lastValidHeading, 0);
    }

    // If it's somewhere generally above the dodecahedron (meaning we should go east for a bit then go straight down)
    // Todo - Make the amount we go to the side scale with how close we were to the dodecahedron's center x-coordinate 
    else 
    {
        // Turning as close to east as we can get
        turnNoRPS(lastValidHeading, 0);
        
        // Going that way for about a second 
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .5);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .5);

        Sleep(1.0);

        // Stopping the motors
        leftMotor.Stop();
        rightMotor.Stop();

        // Turning as close to south as we can get 
        // Makes the assumption that we're currently faced towards zero degrees
        turnNoRPS(0, 270);
    }

    // Drives until we get RPS back (used for all escape cases)
    leftMotor.SetPercent(LEFT_MOTOR_PERCENT);
    rightMotor.SetPercent(RIGHT_MOTOR_PERCENT);

    while (RPS.X() == -1 || RPS.X() == -2) { Sleep(.01); }

    // Waits another second once we get RPS to make sure we're firmly in RPS territory
    Sleep(1.0);

    leftMotor.Stop();
    rightMotor.Stop();
}

// Overloaded method that takes in an (x, y) coordinate instead of a heading
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
        getBackToRPSFromDeadzone(lastValidX, lastValidY, lastValidHeading);

        // Escapes this call of goToPoint because it doesn't really have RPS any more
        return;
    }

    // Todo - Make it start turning even if it doesn't have RPS based on last remembered values so that we don't have to wait for RPS to be valid to start 
    // Generally, turn() is called as part of goToPoint, which can easily make small autocorrections, hence why this threshold doesn't need to be super small   
    while (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 3)
    {
        // * At this point, RPS values are always valid and completely up to date due to the update at the end of the tolerance loop 
        // Because RPS is guaranteed to be valid right now, update the RPS cache 
        updateLastValidRPSValues();

        // Debug 
        SD.Printf("turn: Given currentHeading = %f and endHeading = %f, going through another iteration of the tolerance loop.\r\n", RPS.Heading(), endHeading);    

        // If turning left is quicker 
        if (shouldTurnLeft(RPS.Heading(), endHeading))
        {
            SD.Printf("turn: Given currentHeading = %f and endHeading = %f, robot is turning left.\r\n", RPS.Heading(), endHeading);

            // Todo - If optimizing for time, see how low we can get these thresholds while still being precise enough when it matters 
            // 60+ Degrees Away - Turn as quickly as possible
            if (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 60)
            {
                SD.Printf("turn: Robot is more than 60 degrees away from endHeading. Turning really fast.\r\n");
                leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .6);
                rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .6);
            }

            // 30-60 Degrees Away - Turn quick, but not super quick 
            else if (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 30)
            {
                SD.Printf("turn: Robot is more than 30 degrees away from endHeading. Turning fast, but not super fast.\r\n");
                leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .4);
                rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .4);
            }

            // 0-30 Degrees Away - Turn slowly (precision matters) 
            else 
            {
                SD.Printf("turn: Robot is less than 30 degrees away from endHeading. Turning more slowly.\r\n");
                leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .2);
                rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .2);
            }
        }

        // Otherwise, turning right is quicker 
        else
        {
            SD.Printf("turn: Given currentHeading = %f and endHeading = %f, robot is turning right.\r\n", RPS.Heading(), endHeading);

            // 60+ Degrees Away - Turn as quickly as possible
            if (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 60)
            {
                SD.Printf("turn: Robot is more than 30 degrees away from endHeading. Turning faster.\r\n");
                leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .6);
                rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .6);
            }

            // 30-60 Degrees Away - Turn quick, but not super quick 
            else if (smallestDistanceBetweenHeadings(RPS.Heading(), endHeading) > 30)
            {
                SD.Printf("turn: Robot is more than 30 degrees away from endHeading. Turning faster.\r\n");
                leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .4);
                rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .4);
            }

            // 0-30 Degrees Away - Turn slowly (precision matters) 
            else 
            {
                SD.Printf("turn: Robot is less than 30 degrees away from endHeading. Turning more slowly.\r\n");
                leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .2);
                rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .2);
            }
        }

        Sleep(.01);

        // This function itself is naturally blocking; The return value is only relevant if it's in a deadzone 
        if (loopUntilValidRPS() == -2)
        {
            SD.Printf("goToPoint: Deadzone has become enabled again.\r\n");

            // Causes the program to skip certain goToPoint calls
            hasExhaustedDeadzone = true;

            // Does what you think it does
            SD.Printf("goToPoint: Turning roughly south and going until RPS.\r\n");
            getBackToRPSFromDeadzone(lastValidX, lastValidY, lastValidHeading);

            // Escapes this call of goToPoint because it doesn't really have RPS any more
            return;
        }
    }

    SD.Printf("turn: Given currentHeading = %f, endHeading = %f, robot has turned to within a specified tolerance.\r\n");
    leftMotor.Stop();
    rightMotor.Stop();
}

// Like the normal turn method, but works based off of a last saved heading and an intended heading 
void turnNoRPS(float currentHeading, float endHeading)
{
    // Todo - See if I can up the motor speeds here a little bit to save time (only worth doing if we're at 100 points consistently) 
    // To up the motor speeds, I'd have to figure out Seconds_Per_Degree for every motor speed I want to use and implement them all...
    // That isn't really bad, but also isn't something I want to implement before we have more of the course consistently done 

    if (shouldTurnLeft(currentHeading, endHeading))
    {
        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .25);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .25);
    }

    else 
    {
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .25);
        rightMotor.SetPercent(-LEFT_MOTOR_PERCENT * .25);
    }

    // Degrees * (Seconds / Degrees) = Seconds
    Sleep(smallestDistanceBetweenHeadings(currentHeading, endHeading) * SECONDS_PER_DEGREE);
}

#endif
