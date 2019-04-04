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
 * - Make any aesthetic changes we want to (I'm 100% fine with going past $120 if it's for this. We gotta swank our robot out)
 * - Re-affix the QR code - Implement several points of failure (Tape, Screw, Rubber Cement / Loctite)
 *      - Reminds me - Make sure everything attached to the robot is securely attached (I'm looking at you, Proteus, though we have to make sure the whole thing is good)
 * - Do documentation (/** or JavaDoc) for every function in here
 *
 * */

/* Possible Timesaves (in approximate order of how "safe" or easy to implement they are):
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
 * Today's Ideas:
 *
 * - Implement a "should go faster" parameter to goToPoint that'll basically go twice the speed and take twice the usual tolerance
 * - Implement proportional speeds into goToPoint
 *
 * */

/* Thursday's Todo (Ordered)
 *
 * At open lab from about 4-6:45, so make any optimizations you're going to prior to then
 *
 * - Make sure routine is consistent enough that I'd be happy going into competition with it, then commit
 * - Misc. Time Optimizations
 *      - Implement more extensive speed scaling for goToPoint (works off passed-in parameter that maybe goes from 0-5 for 5 distinct speed modes)
 * - Make goToPoint go faster towards the target when it's still in autocorrection mode (currently goes really slow, actually)
 * - Pathing Optimizations
 *      - DDR - Hardcoded turns, if certain parts are giving us issues?
 *      - Hardcoded turn to get from the end of foosball to roughly token start position, then RPS positioning for actual token? Makes it so we don't have to go down and around (~5s save)
 *
 * */

/*
 * Purchase list for Friday:
 *
 * - Extra rubber bands
 * - Extra cds cell (if budget permits)
 * - Weight distribution fix (probably don't want to do a wrench)
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

/**
 * @brief main is the function that is run at the start of the program. It serves as a hub for most other functions that the program has.
 * @return a value of 0 if everything went right. No other return values currently supported, but theoretically could be in the future.
 */
int main(void)
{
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
    // Navigating to the token drop
    // 13.4, x, 23.3 Before
    SD.Printf("Going to token.\r\n");

    // Approximate, Faster Positioning
    goToPoint(TOKEN_X - 4, TOKEN_Y - 3, false, 0.0, false, 0.0, false, 6);

    // More precise, slower positioning
    goToPoint(TOKEN_X, TOKEN_Y, true, TOKEN_HEADING, false, 0.0, false, 0);

    Sleep(.2); // Giving goToPoint time to "wind down motors" in prep for slower turn
    turnToAngleWhenAlreadyReallyClose(TOKEN_HEADING);

    // Dropping the token
    SD.Printf("Depositing token.\r\n");
    armServo.SetDegree(120);
    Sleep(1.0);
    armServo.SetDegree(30);

    // Go to the side of one of the lights
    SD.Printf("Going to the side of one of the lights.\r\n");
    goToPoint(16, 15, false, 0.0, false, 0.0, false, 6);

    // Go on top of the near light
    SD.Printf("Going on top of the near light.\r\n");
    goToPoint(DDR_BLUE_LIGHT_X - 4.5, DDR_LIGHT_Y, false, 0.0, false, 0.0, false, 2);

    // Reading light sensor output
    leftMotor.Stop();
    rightMotor.Stop();
    SD.Printf("Light Sensor Output: %f\r\n", lightSensor.Value());

    // If the light is blue, do this pathfinding and press the blue button
    if (lightSensor.Value() > .5)
    {
        clearLCD();
        LCD.WriteLine("BLUE.");
        SD.Printf("Light was detected as BLUE.\r\n");

        // Positioning above button
        SD.Printf("Pathfinding above the blue light.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 5, true, 270, false, 0.0, false, 3);
        turnToAngleWhenKindaClose(270);

        // Hitting button for long enough to get bonus goal too
        SD.Printf("Driving into the blue button.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y - 5, false, 0.0, true, 21.0, false, 1);
    }

    // Otherwise, the light is red, so do red button pathfinding and press the red button
    else
    {
        clearLCD();
        LCD.WriteLine("RED.");
        SD.Printf("Light was detected as RED.\r\n");

        // Positioning above button
        SD.Printf("Pathfinding above the red light.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X - 4.75, DDR_LIGHT_Y + 5, true, 270, false, 0.0, false, 3);
        turnToAngleWhenKindaClose(270);

        // Hitting button for long enough to get bonus goal too
        SD.Printf("Driving into the red light.\r\n");
        goToPoint(DDR_BLUE_LIGHT_X - 4.75, DDR_LIGHT_Y - 5, false, 0.0, true, 21.0, false, 1);

        // Getting distance and moving over above blue so it doesn't do weird rotation stuff for the RPS button (this step is unique to red, and is why red takes longer than blue)
        // Angle is so that it turns CCW rather than cw (which tends to hit the blue button)
        // THIS ONE GOES BACKWARDS (~2s faster and theoretically more consistent than old method)
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 5, true, 90, false, 0.0, true, 2);

    }

    // Space and angle for the RPS button
    SD.Printf("Positioning for the RPS button.\r\n");
    goToPoint(RPS_BUTTON_X, RPS_BUTTON_Y, true, RPS_BUTTON_HEADING, false, 0.0, false, 0);
    Sleep(.2); // Giving goToPoint time to "wind down motors"
    turnToAngleWhenAlreadyReallyClose(RPS_BUTTON_HEADING);

    // Press the RPS button
    SD.Printf("Pressing the RPS button.\r\n");

    // This has to be 115+ for it to sleep enough, generally
    // This was 115 for a very long time, I changed this the Tuesday night before final competition
    armServo.SetDegree(125); Sleep(4.0);
    armServo.SetDegree(30);

    // Turning slightly right so that the robot turns right to get to the bottom of the ramp instead of left (and therefore possibly hitting buttons
    turn(90);

    // Move to bottom of ramp
    SD.Printf("Positioning to move up the ramp\r\n");
    goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 2, false, 0.0, false, 0.0, false, 5);

    // Move up ramp and stop somewhere near the top
    // Consistently gets caught on the edge of the ramp but corrects itself (I've never had it fall off the edge irrevocably)
    SD.Printf("Moving up the ramp.\r\n");
    goToPoint(DDR_BLUE_LIGHT_X + 1.8, 57, false, 0.0, false, 0.0, false, 6);

    // Positioning for the foosball task
    if (!hasExhaustedDeadzone)
    {
        SD.Printf("Deadzone still negated. Positioning for foosball.\r\n");
        goToPoint(FOOSBALL_START_X, FOOSBALL_START_Y - .25, true, 7.0, false, 0.0, false, 2);

        Sleep(.3);

        // We want foosball to start out as accurately as possible, basically
        turnToAngleWhenAlreadyReallyClose(7);
    }

    else
    {
        SD.Printf("Deadzone is in effect. Could not position initially for foosball.\r\n");
    }

    armServo.SetDegree(95);
    Sleep(.5); // Giving the servo time to get down

    SD.Printf("Deadzone still negated. Performing foosball.\r\n");

    // goToPointFoosball();

    if (!hasExhaustedDeadzone)
    {
        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .4);
        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .4);

        Sleep(1.9);

        leftMotor.Stop();
        rightMotor.Stop();

        armServo.SetDegree(75);
        Sleep(.5);

        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .2);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .2);

        Sleep(.75);

        leftMotor.Stop();
        rightMotor.Stop();

        armServo.SetDegree(95);
        Sleep(.5);

        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .2);
        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .2);

        Sleep(.75);

        leftMotor.Stop();
        rightMotor.Stop();

        armServo.SetDegree(30);
        Sleep(.5);

        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .4);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .4);

        Sleep(1.75);

        leftMotor.Stop();
        rightMotor.Stop();
    }

    else
    {
        SD.Printf("Deadzone is in effect. Could not actually go backwards while doing foosball.\r\n");
    }

    // Going to the left part
    if (!hasExhaustedDeadzone)
        goToPoint(20, 48, false, 0.0, false, 0.0, false, 6);

    if (!hasExhaustedDeadzone)
        goToPoint(8, 48, false, 0.0, false, 0.0, false, 6);

    // Positioning for the lever
    // Approximate, faster positioning most of the way there
    if (!hasExhaustedDeadzone)
    {
        goToPoint(LEVER_X + 2, LEVER_Y - 4, false, 0.0, false, 0.0, false, 5);
    }

    // More precise, slower positioning once we're nearly there
    if (!hasExhaustedDeadzone)
    {
        SD.Printf("Deadzone still negated. Positioning for lever.\r\n");
        goToPoint(LEVER_X, LEVER_Y, true, LEVER_HEADING, false, 1.5, false, 0);
    }

    if (!hasExhaustedDeadzone)
    {
        turnToAngleWhenAlreadyReallyClose(LEVER_HEADING);
    }

    SD.Printf("Pressing lever, even if deadzone is now in effect.\r\n");
    armServo.SetDegree(110);
    Sleep(1.0);
    armServo.SetDegree(30);

    // If it hits the deadzone, it should skip to here within a few seconds after it gets back to RPS
    SD.Printf("Going to the top of the return ramp.\r\n");
    goToPoint(6, 55.0, false, 0.0, false, 0.0, false, 6);

    SD.Printf("Hitting the end button.\r\n");
    goToPoint(5.0, 5.0, false, 0.0, false, 0.0, false, 6);
}


