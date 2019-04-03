#ifndef CUSTOMUTILITY_H
#define CUSTOMUTILITY_H

#include "conversions.h"
#include "constants.h"

using namespace std;

/**
 * @brief getDistance is just distance formula. It's worth noting that most units will already be in inches because that's what RPS reports in.
 * @param x1 is the first x coordinate.
 * @param y1 is the first y coordinate.
 * @param x2 is the second x coordinate.
 * @param y2 is the second y coordinate.
 * @return The length, in inches, of a line if it were to be drawn directly in between the two points (AKA distance between points, but fancy)
 */
float getDistance(float x1, float y1, float x2, float y2) { return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)); }

/**
 * @brief clearLCD does exactly what you think it does - Clears the LCD screen.
 */
void clearLCD() { LCD.Clear(FEHLCD::Black); LCD.SetFontColor(FEHLCD::White); }

/**
 * @brief smallestDistanceBetweenHeadings reports the smallest heading difference between two headings. Works by calculating clockwise and counterclockwise distances and making a decision based on which is smaller.
 * @param startHeading is the first heading - Arbitrary choice which is start and end, but the robot's heading is generally the startHeading
 * @param endHeading is the second heading - Arbitrary choice which is start and end, but the heading we're trying to turn to is generally the endHeading
 * @return
 */
float smallestDistanceBetweenHeadings(float startHeading, float endHeading)
{
    float cwDistance, ccwDistance;

    // Clockwise options
    if (startHeading > endHeading) { cwDistance = startHeading - endHeading; } // CW, No-Wrap
    else { cwDistance = (0 + startHeading) + (360 - endHeading); } // CW, Wrap

    // Counterclockwise options
    if (startHeading > endHeading) { ccwDistance = (0 + endHeading) + (360 - startHeading); } // CCW, Wrap
    else { ccwDistance = endHeading - startHeading; }

    if (cwDistance <= ccwDistance) 
        return cwDistance; 
    return ccwDistance;
}

void gradualServoTurn(float endDegree)
{
    float currentDegree = 30;
    while (currentDegree <= endDegree)
    {
        armServo.SetDegree(currentDegree);
        currentDegree++;
        Sleep(.01);
    }
    Sleep(.5);
    armServo.SetDegree(30);
    Sleep(.5);
}

/**
 * @brief getDesiredHeading calculates and returns the angle necessary in order to point from point (x1, y1) to point (x2, y2).
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return
 *
 * Note: Angles are the same as the unit circle, with "North" on our course being 90 Degrees.
 */
float getDesiredHeading(float x1, float y1, float x2, float y2)
{
    float x_dot = x2 - x1;
    float y_dot = y2 - y1;

    // 0 to 90
    if (x_dot > 0 && y_dot > 0)
        return 0 + radianToDegree(atan(abs(y_dot) / abs(x_dot)));

    // 90 to 180
    else if (x_dot <= 0 && y_dot > 0)
        return 90 + radianToDegree(atan(abs(x_dot) / abs(y_dot)));

    // 180 to 270
    else if (x_dot <= 0 && y_dot <= 0)
        return 180 + radianToDegree(atan(abs(y_dot) / abs(x_dot)));

    // 270 to 360
    else
        return 270 + radianToDegree(atan(abs(x_dot) / abs(y_dot)));
}

bool shouldTurnLeft(float startHeading, float endHeading)
{
    float cwDistance, ccwDistance;

    if (startHeading > endHeading) { cwDistance = startHeading - endHeading; } // CW No-Wrap
    else { cwDistance = (0 + startHeading) + (360 - endHeading); } // CW Wrap

    if (startHeading > endHeading) { ccwDistance = (0 + endHeading) + (360 - startHeading); } // CCW Wrap
    else { ccwDistance = endHeading - startHeading; } // CCW No-Wrap

    // Debug, because this function is apparently still bugging out even though I've tested it extensively
    /*
    if (ccwDistance <= cwDistance)
        SD.Printf("shouldTurnLeft: Given startHeading %f and endHeading %f, robot should turn left.\r\n", startHeading, endHeading);
    else
        SD.Printf("shouldTurnLeft: Given startHeading %f and endHeading %f, robot should turn right.\r\n", startHeading, endHeading);
    */

    // If ccw is less than cw, return that it should turn left. Otherwise, return false
    return (ccwDistance <= cwDistance);
}

void loopUntilTouch()
{
    float x, y;
    while (!LCD.Touch(&x, &y))
    {
        SD.Printf("Waiting for screen touch to progress in the program\r\n");
        clearLCD();
        LCD.WriteLine("Waiting for Screen Touch.");
        Sleep(.1);
    }
}

#endif // CUSTOMUTILITY_H
