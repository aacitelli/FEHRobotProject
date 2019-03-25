#ifndef CUSTOMUTILITY_H
#define CUSTOMUTILITY_H

#include <FEHRPS.h>

using namespace std;

#define PI 3.14159265358979323846
float radianToDegree(float radianValue) { return radianValue * (180.0 / PI); }
float degreeToRadian(float degreeValue) { return degreeValue * (PI / 180.0); }
float getDistance(float x1, float y1, float x2, float y2) { return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)); }

void clearLCD() { LCD.Clear(FEHLCD::Black); LCD.SetFontColor(FEHLCD::White); }

/**
 * @brief loopWhileNoRPS is called in certain situations right before important calculations to make sure those
 * calculations are done with as up-to-date RPS measurements as possible. It loops until RPS returns non-error-code values.
 */
void loopWhileNoRPS()
{
    while (RPS.X() == -1 || RPS.X() == -2 || RPS.Y() == -1 || RPS.Y() == -2 || RPS.Heading() == -1 || RPS.Heading() == -2) { Sleep(.001); } return;
}

// The QR code sits basically on top of the centroid but the code does account correctly for if there's a distance differential here
const float DISTANCE_BETWEEN_RPS_AND_CENTROID = 0;

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

// Note - This picks whichever distance is less than 180 b/c that's the only case we care about when this function is used
// This function doesn't care about which is the start and end, but the code copied over from shouldTurnLeft was way faster
// Todo - Make this simpler b/c I initially just used the code from shouldTurnLeft - Current iteration is needlessly processor-intensive (though the impact is negligible)
float smallestDistanceBetweenHeadings(float startHeading, float endHeading)
{
    float cwDistance, ccwDistance;

    // Calculating both cases for each direction
    if (startHeading > endHeading) { cwDistance = startHeading - endHeading; } // CW, No-Wrap
    else { cwDistance = (0 + startHeading) + (360 - endHeading); } // CW, Wrap
    if (startHeading > endHeading) { ccwDistance = (0 + endHeading) + (360 - startHeading); } // CCW, Wrap
    else { ccwDistance = endHeading - startHeading; }

    // Returns minimum of cw and ccw directions
    if (cwDistance <= ccwDistance) return cwDistance; return ccwDistance;
}

float getDesiredHeading(float x1, float y1, float x2, float y2)
{
    float x_dot = x2 - x1;
    float y_dot = y2 - y1;

    float returnValue;

    // 0 to 90 Degrees (Top-Right)
    if (x_dot > 0 && y_dot > 0)
        returnValue = 0 + radianToDegree(atan(abs(y_dot) / abs(x_dot)));

    // 90 to 180 Degrees (Top-Left)
    else if (x_dot <= 0 && y_dot > 0)
        returnValue = 90 + radianToDegree(atan(abs(x_dot) / abs(y_dot)));

    // 180 to 270 Degrees (Bottom-Left)
    else if (x_dot <= 0 && y_dot <= 0)
        returnValue = 180 + radianToDegree(atan(abs(y_dot) / abs(x_dot)));

    // 270 to 360 Degrees (Bottom-Right)
    else
        returnValue = 270 + radianToDegree(atan(abs(x_dot) / abs(y_dot)));

    return returnValue;
}

#endif // CUSTOMUTILITY_H