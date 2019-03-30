#ifndef CONVERSIONS_H
#define CONVERSIONS_H

// Imports
#include <FEHRPS.h>
// Custom Libraries
// Todo - Trim these down to what we actually need

// Needed for sin/cos, etc.
using namespace std;

// Radian <-> Degree
float radianToDegree(float radianValue) { return radianValue * (180.0 / PI); }
float degreeToRadian(float degreeValue) { return degreeValue * (PI / 180.0); }

// If there's distance between QR code and centroid, this is implemented into the relevant functions 
float rpsXToCentroidX()
{
    // 0 Degrees (Inclusive) to 90 Degrees (Exclusive)
    if (RPS.Heading() >= 0 && RPS.Heading() < 90)
        return RPS.X() + DISTANCE_BETWEEN_RPS_AND_CENTROID * cos(degreeToRadian(RPS.Heading()));

    // 90 Degrees (Inclusive) to 180 Degrees (Exclusive)
    else if (RPS.Heading() >= 90 && RPS.Heading() < 180)
        return RPS.X() - (DISTANCE_BETWEEN_RPS_AND_CENTROID * sin(degreeToRadian(RPS.Heading() - 90)));

    // 180 Degrees (Inclusve) to 270 Degrees (Exclusive)
    else if (RPS.Heading() >= 180 && RPS.Heading() < 270)
        return RPS.X() - (DISTANCE_BETWEEN_RPS_AND_CENTROID * cos(degreeToRadian(RPS.Heading() - 180)));

    // 270 Degrees (Inclusive) to 360 Degrees (Inclusive)
    else
        return RPS.X() + (DISTANCE_BETWEEN_RPS_AND_CENTROID * sin(degreeToRadian(RPS.Heading() - 270)));
}

// If there's distance between QR code and centroid, this is implemented into the relevant functions 
float rpsYToCentroidY()
{
    // 0 Degrees (Inclusive) to 90 Degrees (Exclusive)
    if (RPS.Heading() >= 0 && RPS.Heading() < 90)
        return RPS.Y() + (DISTANCE_BETWEEN_RPS_AND_CENTROID * sin(degreeToRadian(RPS.Heading())));

    // 90 Degrees (Inclusive) to 180 Degrees (Exclusive)
    else if (RPS.Heading() >= 90 && RPS.Heading() < 180)
        return RPS.Y() + (DISTANCE_BETWEEN_RPS_AND_CENTROID * cos(degreeToRadian(RPS.Heading() - 90)));

    // 180 Degrees (Inclusive) to 270 Degrees (Exclusive)
    else if (RPS.Heading() >= 180 && RPS.Heading() < 270)
        return RPS.Y() - (DISTANCE_BETWEEN_RPS_AND_CENTROID * sin(degreeToRadian(RPS.Heading() - 180)));

    // 270 Degrees (Inclusive) to 360 Degrees (Inclusive)
    else
        return RPS.Y() - (DISTANCE_BETWEEN_RPS_AND_CENTROID * cos(degreeToRadian(RPS.Heading() - 270)));
}

// Takes in an angle and returns whatever's 180 degrees from it, accounting for overflow for high angles
float rotate180Degrees(float degrees)
{
    // Taking modulus by 360 accounts for the fact that 380 Degrees = 20 Degrees (not specifically that, but that general kind of thing)
    // fmod is just modulus but for two float values, which we need here
    return fmod((degrees + 180), 360.0);
}

#endif
