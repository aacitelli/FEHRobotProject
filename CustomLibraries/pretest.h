#ifndef SETUP_H
#define SETUP_H

#include <FEHRPS.h>
#include "rps.h"
#include "utility.h"

// Imports

// Initializing requisite systems
void init()
{
    SD.Printf("Running initialization protocols.\r\n");
    RPS.InitializeTouchMenu();
    SD.OpenLog();
}

// Gets RPS Coordinates - Used to basically negate the minor differences in each course 
// Order: Token -> Far DDR Butotn -> RPS Button -> Foosball Start -> Lever (to the right is better positioning, I think)
void calibrate()
{
    SD.Printf("Running initialization procedure.\r\n");

    // Token
    loopUntilTouch();
    loopUntilValidRPS();
    TOKEN_X = RPS.X();
    TOKEN_Y = RPS.Y();
    TOKEN_HEADING = RPS.Heading();
    SD.Printf("Token X: %f\r\n", TOKEN_X);
    SD.Printf("Token Y: %f\r\n", TOKEN_Y);
    SD.Printf("Token Heading: %f\r\n", TOKEN_HEADING);
    Sleep(1.0);

    // DDR Blue (Far) Button
    loopUntilTouch();
    loopUntilValidRPS();
    DDR_BLUE_LIGHT_X = RPS.X();
    DDR_LIGHT_Y = RPS.Y();
    SD.Printf("DDR Blue X: %f\r\n", DDR_BLUE_LIGHT_X);
    SD.Printf("DDR Y: %f\r\n", DDR_LIGHT_Y);
    Sleep(1.0);

    // RPS Button
    loopUntilTouch();
    loopUntilValidRPS();
    RPS_BUTTON_X = RPS.X();
    RPS_BUTTON_Y = RPS.Y();
    RPS_BUTTON_HEADING = RPS.Heading();
    SD.Printf("RPS Button X: %f\r\n", RPS_BUTTON_X);
    SD.Printf("RPS Button Y: %f\r\n", RPS_BUTTON_Y);
    SD.Printf("RPS Button Heading: %f\r\n", RPS_BUTTON_HEADING);
    Sleep(1.0);

    // Foosball Start
    // Todo - Measure how far the right side is from the left side to eliminate a sampling point 
    loopUntilTouch();
    loopUntilValidRPS();
    FOOSBALL_START_X = RPS.X();
    FOOSBALL_START_Y = RPS.Y();
    SD.Printf("Foosball Start X: %f\r\n", FOOSBALL_START_X);
    SD.Printf("Foosball Start Y: %f\r\n", FOOSBALL_START_Y);
    Sleep(1.0);

    // Foosball width is constant across courses, so we can just apply an offset to the start position to get the end position
    const float FOOSBALL_HORIZONTAL_DISTANCE = 8;
    FOOSBALL_END_X = FOOSBALL_START_X - FOOSBALL_HORIZONTAL_DISTANCE;

    // Making the logical assumption that start and end are at the same height (as they need to be for the robot to go 180 degrees backwards, parallel to foosball)
    FOOSBALL_END_Y = FOOSBALL_START_Y;

    // Lever
    // Todo - Figure out the best place to do lever from 
    loopUntilTouch();
    loopUntilValidRPS();
    LEVER_X = RPS.X();
    LEVER_Y = RPS.Y();
    LEVER_HEADING = RPS.Heading();
    SD.Printf("Lever X: %f\r\n", LEVER_X);
    SD.Printf("Lever Y: %f\r\n", LEVER_Y);

    // Preparation for next program step
    armServo.SetDegree(30);
    clearLCD();
}

#endif
