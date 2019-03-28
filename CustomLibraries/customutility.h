#ifndef CUSTOMUTILITY_H
#define CUSTOMUTILITY_H

#include <FEHRPS.h>

using namespace std;

#define PI 3.14159265358979323846

// Radian <-> Degree Conversion Functions
float radianToDegree(float radianValue) { return radianValue * (180.0 / PI); }
float degreeToRadian(float degreeValue) { return degreeValue * (PI / 180.0); }

// Distance Formula
float getDistance(float x1, float y1, float x2, float y2) { return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)); }

// Used for debug
void clearLCD() { LCD.Clear(FEHLCD::Black); LCD.SetFontColor(FEHLCD::White); }

// Infinite loop while RPS evaluates to either -1 or -2
bool RPSIsBad() { return (RPS.X() == -1 || RPS.Y() == -1 || RPS.Heading() == -1 || RPS.X() == -2 || RPS.Y() == -2 || RPS.Heading() == -2); }
void loopWhileNoRPS()
{
    while (RPSIsBad())
    {
        Sleep(.01);
    }
}

// The QR code sits basically on top of the centroid but the code does account correctly for if there's a distance differential here
const float DISTANCE_BETWEEN_RPS_AND_CENTROID = 0;

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

// Note - This picks whichever distance is less than 180 b/c that's the only case we care about when this function is used
// This function doesn't care about which is the start and end, but the code copied over from shouldTurnLeft was way faster
// Todo - Make this simpler b/c I initially just used the code from shouldTurnLeft - Current iteration is needlessly processor-intensive (though the impact is negligible)
float smallestDistanceBetweenHeadings(float startHeading, float endHeading)
{
    float cwDistance, ccwDistance;
    if (startHeading > endHeading) { cwDistance = startHeading - endHeading; } // CW, No-Wrap
    else { cwDistance = (0 + startHeading) + (360 - endHeading); } // CW, Wrap
    if (startHeading > endHeading) { ccwDistance = (0 + endHeading) + (360 - startHeading); } // CCW, Wrap
    else { ccwDistance = endHeading - startHeading; }
    if (cwDistance <= ccwDistance) return cwDistance; return ccwDistance;
}

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

#endif // CUSTOMUTILITY_H
