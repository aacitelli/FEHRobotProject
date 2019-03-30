#ifndef CUSTOMUTILITY_H
#define CUSTOMUTILITY_H

#include <FEHRPS.h>

using namespace std;

// Distance Formula
float getDistance(float x1, float y1, float x2, float y2) { return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)); }

// Used for debug
void clearLCD() { LCD.Clear(FEHLCD::Black); LCD.SetFontColor(FEHLCD::White); }

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

    if (cwDistance <= ccwDistance) 
        return cwDistance; 
    return ccwDistance;
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

void loopUntilTouch()
{
    float x, y;
    while (!LCD.Touch(&x, &y))
    {
        clearLCD();
        LCD.WriteLine("Waiting for Screen Touch.");
        Sleep(.1);
    }
}

#endif // CUSTOMUTILITY_H
