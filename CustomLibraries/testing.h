#ifndef TESTING_H
#define TESTING_H

// Imports
#include <FEHRPS.h> // RPS functions
#include <utility.h> // clearLCD()

/**
 * @brief rpsSquare takes the robot in an approximate square. This isn't hardcoded RPS coordinates, though - Just 6 inches in the cardinal directions from were the robot current is.
 */
void rpsSquare()
{
    while (true)
    {
        goToPoint(10, 16, false, 0.0, false, 0.0, false);
        goToPoint(16, 16, false, 0.0, false, 0.0, false);
        goToPoint(16, 10, false, 0.0, false, 0.0, false);
        goToPoint(10, 10, false, 0, false, 0.0, false);
    }
}

/**
 * @brief rpsTest loops forever, reporting RPS values on-screen until it is forcibly powered down.
 */
void rpsTest()
{
    while (true)
    {
        LCD.WriteLine(RPS.X());
        LCD.WriteLine(RPS.Y());
        LCD.WriteLine(RPS.Heading());

        Sleep(.1);
        clearLCD();
    }
}

#endif // TESTING_H
