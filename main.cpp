// FEH-Specific Libraries
#include <FEHLCD.H> // Necessary for during-test debug information
#include <FEHSD.h> // Necessary for post-test debug information
#include <FEHBattery.h> // Necessary for voltage check at beginning

// C/C++ Libaries
#include <cmath>
#include <stdlib.h>
#include <ctime>

// Custom Libraries
// Todo - Trim these down to what we actually need
#include "CustomLibraries/constants.h"
#include "CustomLibraries/posttest.h"
#include "CustomLibraries/pretest.h"
#include "CustomLibraries/navigation.h"
#include "CustomLibraries/testing.h"

/* NEXT WEEK'S TODO LIST
 *
 * HARDWARE (Loosely prioritized)
 *
 * - Make sure everything relevant is covered in tape
 * - Make any aesthetic changes we want to (I'm 100% fine with going past $120 if it's for this, we gotta swank our robot out)
 * - Re-affix the QR code - Implement several points of failure (Tape, Screw, Rubber Cement / Loctite)
 *      - Reminds me - Make sure everything attached to the robot is securely attached (I'm looking at you, Proteus, though we have to make sure the whole thing is good)
 *
 * SOFTWARE (Highest Priority -> Lowest Priority)
 *
 * - Figure out if high tolerances with goToPoint are a good thing
 *      - I'm still not sure if the high tolerances are what's causing the offset, or if low tolerances are still causing overshoots and correction
 *          - We just need testing to figure this out
 * - Getting closer and closer to 100 points consistently is the priority
 *      - Slight tuning to make it decently fast but at the same time precise enough to work consistently is mostly what we need
 *      - No major overhauls necessary, though slight consistency reworks could be necessary
 *      - Might be a case of sacrificing speed for consistency, which I'm fine with, but beyond a certain point we legit just won't finish the course
 * - Make goToPoint go slower when it gets close (to avoid inaccuracy when going close quick)
 *      - If we go with this (probably will) it's probably best to implement lower tolerances because this will ensure that we're being precise when it matters
 * - Do documentation (/**) for every function in here
 *
 * MISC
 *
 * - All the website and documentation stuff
 * - Any robot assignments that are due
 * - Individual competition reflection (I'm assuming that's due)
 *
 * */

/* Possible Timesaves (in approximate order of how "preferable" they are):
 *
 * - Tweak turn() motor amounts to be faster (sacrificing a little bit of precision for speed - I think it's worth it, and I'll probably end up implementing this)
 *      - "Too much" is whenever it overshoots the end tolerance for long enough that the slower turn would overpass it
 *      - "Too ligtle" is whenever it hits it consistently every time but could still be faster while achieving the same result
 * - Raise the turn() angle tolerance because it's always run as part of goToPoint and that'll autocorrect for small inaccuracies while being faster overall
 * - Raise goToPoing passed-in speeds on segments that are consistent - Can shave probably a second off of a lot of these
 *      - This just means modifying the constants in the goToPoint calls - Not a hard fix, but could cause instability and unreliability 
 * - If we end up making goToPoint slower when closer, tweak that downshift to be later on in the timing (closer to the end point) so it gets slow as late as possible while still being precise enough
 *      - Basically, keep the current system, but if the tolerance is greater, go on average a lot faster through the entire procedure
 *
 * */

using namespace std;

void init(); void deinit();
void loopWhileStartLightIsOff();
void performanceTest4();
void finalRoutine();
void rpsTest();
void updateLastValidRPSValues();
void turnSouthAndGoUntilRPS(float startHeading);
void calibrate();

void foosball()
{
    armServo.SetDegree(95);

    Sleep(.5);

    leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .3);
    rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .3);

    Sleep(2.0);

    leftMotor.Stop();
    rightMotor.Stop();
}

/**
 * @brief main is the function that is run at the start of the program. It serves as a hub for most other functions that the program has.
 * @return a value of 0 if everything went right. No other return values currently supported, but theoretically could be in the future.
 */
int main(void)
{
    // foosball();

    // Initializes RPS & SD Card
    init();

    // Testing Procedures (leave commented out unless using)
    // rpsSquare();
    // rpsTest();

    // Calibration procedure
    calibrate();

    // This is where we put the token in
    armServo.SetDegree(30);

    // TODO - Implement final touch here when we prompt

    // Sensing the start light, automatically triggering if it takes more than 30 seconds
    int iterationCount = 0;
    while (lightSensor.Value() > .45 && iterationCount < 300)
    {
        iterationCount++;

        clearLCD();
        LCD.WriteLine("Waiting for start light.");
        SD.Printf("Waiting for start light.\r\n");

        Sleep(.1);
    }

    // Runs any code from the current routine
    finalRoutine();

    // Shuts down whatever needs shut down at the end of a run
    deinit();

    // Exit code 0, indicating early termination did not occur
    return 0;
}

/**
 * @brief finalRoutine is the chain of goToPoint (and other misc. function) calls that make up our final competition run.
 */
void finalRoutine()
{
    // TODO - Figure out if there's any optimizations we can take to have certain goToPoint method calls go faster, or go backwards, or anything like that where you change up the parameters

    // Navigating to the token drop
    // 13.4, x, 23.3 Before
    SD.Printf("Going to token.\r\n");
    goToPoint(TOKEN_X, TOKEN_Y, true, TOKEN_HEADING, false, 0.0, false);
    turnToAngleWhenAlreadyReallyClose(TOKEN_HEADING);

    // Dropping the token
    SD.Printf("Depositing token.\r\n");
    armServo.SetDegree(120);
    Sleep(1.0);
    armServo.SetDegree(30);

    // Go to the side of one of the lights
    SD.Printf("Going to the side of one of the lights.\r\n");
    goToPoint(16, 15, false, 0.0, false, 0.0, false);

    // Go on top of the near light
    SD.Printf("Going on top of the near light.\r\n");
    goToPoint(DDR_BLUE_LIGHT_X - 5.25, DDR_LIGHT_Y, false, 0.0, false, 0.0, false);

    // Reading light sensor output
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(.5); // Todo - Remove this in end test
    SD.Printf("Light Sensor Output: %f\r\n", lightSensor.Value());

    // If the light is blue, do this pathfinding and press the blue button
    if (lightSensor.Value() > .5)
    {
        clearLCD();
        LCD.WriteLine("BLUE.");
        SD.Printf("Light was detected as BLUE.\r\n");

        // Positioning above button
        SD.Printf("Pathfinding above the blue light.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 5, true, 270, false, 0.0, false);

        // Hitting button for long enough to get bonus goal too
        SD.Printf("Driving into the blue button.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y - 5, false, 0.0, true, 15.0, false);
    }

    // Otherwise, the light is red, so do red button pathfinding and press the red button
    else
    {
        clearLCD();
        LCD.WriteLine("RED.");
        SD.Printf("Light was detected as RED.\r\n");

        // Positioning above button
        SD.Printf("Pathfinding above the red light.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X - 5.25, DDR_LIGHT_Y + 5, true, 270, false, 0.0, false);

        // Hitting button for long enough to get bonus goal too
        SD.Printf("Driving into the red light.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X - 5.25, DDR_LIGHT_Y - 5, false, 0.0, true, 15.0, false);
    }

    // Space and angle for the RPS button
    SD.Printf("Positioning for the RPS button.\r\n");
    goToPoint(RPS_BUTTON_X, RPS_BUTTON_Y, true, RPS_BUTTON_HEADING, false, 0.0, false);
    turnToAngleWhenAlreadyReallyClose(RPS_BUTTON_HEADING);

    // Press the RPS button
    SD.Printf("Pressing the RPS button.\r\n");

    // This has to be 115+ for it to sleep enough
    armServo.SetDegree(115); Sleep(4.0);
    armServo.SetDegree(30);

    // Move to bottom of ramp
    SD.Printf("Positioning to move up the ramp\r\n");
    goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 2, false, 0.0, false, 0.0, false);

    // Move up ramp and stop somewhere near the top
    SD.Printf("Moving up the ramp.\r\n");
    goToPoint(DDR_BLUE_LIGHT_X + 2, 55, false, 0.0, false, 0.0, false);

    // Positioning for the foosball task
    if (!hasExhaustedDeadzone)
    {
        SD.Printf("Deadzone still negated. Positioning for foosball.\r\n");
        goToPoint(FOOSBALL_START_X, FOOSBALL_START_Y, true, 0.0, false, 0.0, false);

        // TODO - Probably implement the more precise turning method here, especially w/ how the modified goToPoint works
    }

    else
    {
        SD.Printf("Deadzone is in effect. Could not position initially for foosball.\r\n");
    }

    // Todo - Figure out the precise angle we need so that it presses down hard enough on foosball to move it, but easily enough that it doesn't lift our vehicle (consider screwing around w/ weight distribution stuff)
    // Pressing the lever setup on top of foosball  - SetDegree locks it in that place and will return it if forced upward
    SD.Printf("Pressing down on foosball, even if a deadzone is still in effect.\r\n");
    armServo.SetDegree(95);
    Sleep(.5);

    // Does the movement involved in foosball (moves straight west)
    if (!hasExhaustedDeadzone)
    {
        SD.Printf("Deadzone still negated. Performing foosball.\r\n");

        // Custom-designed method just for foosball that basically uses a x-coord cutoff rather than a hardcoded tolerance check
        goToPointFoosball();
    }

    else
    {
        SD.Printf("Deadzone is in effect. Could not actually go backwards while doing foosball.\r\n");
    }

    // Returning the arm back to default position after foosball so it doesn't get in the way
    SD.Printf("Foosball done. Raising arm back up.\r\n");
    armServo.SetDegree(30);
    Sleep(.5);

    // Going to the left part
    if (!hasExhaustedDeadzone)
        goToPoint(20, 48, false, 0.0, false, 0.0, false);

    if (!hasExhaustedDeadzone)
        goToPoint(8, 48, false, 0.0, false, 0.0, false);

    // Position for the lever
    // Todo - Decide if we want to approach head-on or from the side (from the side would probably be more consistent but would require slightly more precise positioning, I guess
    if (!hasExhaustedDeadzone)
    {
        SD.Printf("Deadzone still negated. Positioning for lever.\r\n");
        goToPoint(LEVER_X, LEVER_Y, true, LEVER_HEADING, false, 1.5, false);
    }

    else
    {
        SD.Printf("Deadzone is back in effect. Could not position for lever.\r\n");
    }

    SD.Printf("Pressing lever, even if deadzone is now in effect.\r\n");
    armServo.SetDegree(110);
    Sleep(1.0);
    armServo.SetDegree(30);

    // If it hits the deadzone, it should skip to here within a few seconds after it gets back to RPS
    SD.Printf("Going to the top of the return ramp.\r\n");
    goToPoint(6, 55.0, false, 0.0, false, 0.0, false);

    SD.Printf("Hitting the end button.\r\n");
    goToPoint(6.0, 6.0, false, 0.0, false, 0.0, false);
}


