#ifndef RPS_H
#define RPS_H

// Imports
#include <FEHRPS.h>

// Updates global variables, but only to "valid" vales (anything that's not "no rps" or a deadzone value)
void updateLastValidRPSValues()
{
    if (RPS.X() != -1 && RPS.X() != -2)
        lastValidX = RPS.X();
    if (RPS.Y() != -1 && RPS.Y() != -2)
        lastValidY = RPS.Y();
    if (RPS.Heading() != -1 && RPS.Heading() != -2)
        lastValidHeading = RPS.Heading();
}

// Sensing invalid RPS 
int rpsState() 
{ 
    if (RPS.X() == -1 || RPS.Y() == -1 || RPS.Heading() == -1)
        return -1;
    else if (RPS.X() == -2 || RPS.Y() == -2 || RPS.Heading() == -2)
        return -2;
    return 0;
}

// Return value of 0 indicates valid operation, -2 indicates it's in a deadzone 
int loopUntilValidRPS()
{
    int iterations = 0;
    while (rpsState() == -1 || rpsState() == -2)
    {
        if (rpsState() == -2)
        {
            return -2;
        }

        iterations++;
        SD.Printf("Current iterations looping for RPS: %d\r\n", iterations);

        Sleep(.01);
    }

    // Getting through that loop and not returning by this point indicates that it now has RPS 
    return 0;
}

#endif
