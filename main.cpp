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

/* Friday's Todo (Ordered)
 *
 * - Replace wrench if at all possible
 * - Make sure consistency is alright
 *      - Test RPS alignment, see if we need to consistently angle offset differently than we already do
 * - See if there's any good alternative to hitting the lever the way we currently do
 * - Add a checkpoint near the top of the map so that it goes up the middle of the ramp and doesn't get really close to the side like it currently does
 * - Go backwards to the lever around the top instead of the bottom (Probably ~5s quicker)
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

/*
 * Competition Todo:
 *
 * Get code ready for lighting checks and figure out what we need to measure
 *
 */

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
    /* Navigating to the token drop */
    // Approximate, Faster Positioning
    goToPoint(TOKEN_X - 4, TOKEN_Y - 3, false, 0.0, false, 0.0, false, 6);

    // More precise, slower positioning
    goToPoint(TOKEN_X, TOKEN_Y, true, TOKEN_HEADING, false, 0.0, false, 0);

    // Small wind-down time so that the next method run starts with an accurate heading
    Sleep(.2);

    // Turns slowly, but really precisely
    turnToAngleWhenAlreadyReallyClose(TOKEN_HEADING);

    // Dropping the token
    armServo.SetDegree(120);
    Sleep(1.0);
    armServo.SetDegree(30);

    // Go to the side of one of the lights so that we can correctly align onto the close button
    goToPoint(16, 15, false, 0.0, false, 0.0, false, 6);

    // Go on top of the near light
    goToPoint(DDR_BLUE_LIGHT_X - 4.5, DDR_LIGHT_Y, false, 0.0, false, 0.0, false, 2);

    // Reading light sensor output
    leftMotor.Stop();
    rightMotor.Stop();
    SD.Printf("Light Sensor Output: %f\r\n", lightSensor.Value());

    // If the light is blue, do this pathfinding and press the blue button
    if (lightSensor.Value() > .5)
    {
        // Positioning approximately above the blue button
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 5, true, 270, false, 0.0, false, 3);

        // Was having consistency issues with being straight enough, so this one should reduce the angle we're currently at from like += 10 degrees to += 5
        turnToAngleWhenKindaClose(270);

        // Driving down into the blue button
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y - 5, false, 0.0, true, 23.0, false, 1);
    }

    // Otherwise, the light is red, so do red button pathfinding and press the red button
    else
    {
        // Positioning above button
        goToPoint(DDR_BLUE_LIGHT_X - 4.75, DDR_LIGHT_Y + 5, true, 270, false, 0.0, false, 3);

        // Was having consistency issues with being straight enough, so this one should reduce the angle we're currently at from like += 10 degrees to += 5
        turnToAngleWhenKindaClose(270);

        // Hitting button for long enough to get bonus goal too
        goToPoint(DDR_BLUE_LIGHT_X - 4.75, DDR_LIGHT_Y - 5, false, 0.0, true, 23.0, false, 1);

        // In the case of red, we have to back up a little bit so that we don't turn into the blue button when aligning for the RPS button
        goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 5, true, 90, false, 0.0, true, 2);

    }

    // Space and angle for the RPS button
    goToPoint(RPS_BUTTON_X, RPS_BUTTON_Y, true, RPS_BUTTON_HEADING, false, 0.0, false, 0);

    // Giving goToPoint time to "wind down motors"
    Sleep(.2);

    // Making sure the angle for the RPS button is super accurate
    turnToAngleWhenAlreadyReallyClose(RPS_BUTTON_HEADING);

    // Physically pressing the RPS button
    armServo.SetDegree(125); Sleep(4.0);
    armServo.SetDegree(30);

    // So that the robot turns right to get to the bottom of the ramp and not the left (where it runs the risk of hitting the blue button)
    turn(90);

    // Move to bottom of ramp
    goToPoint(DDR_BLUE_LIGHT_X, DDR_LIGHT_Y + 2, false, 0.0, false, 0.0, false, 5);

    // Move up ramp and stop somewhere near the top nearish to foosball
    // TODO - Add an additional checkpoint here so that it doesn't occasionally catch
    goToPoint(DDR_BLUE_LIGHT_X + 2, 40, false, 0.0, false, 0.0, false, 5);
    goToPoint(DDR_BLUE_LIGHT_X + 1.8, 57, false, 0.0, false, 0.0, false, 5);

    // Past this point, this check needs to be here for basically every call so if it loses deadzone it skips all the way to the end
    if (!hasExhaustedDeadzone)
    {
        // Positions for foosball itself
        goToPoint(FOOSBALL_START_X, FOOSBALL_START_Y - .25, true, 7.0, false, 0.0, false, 2);

        // Makes sure the motors are caught up so that the specific angle check is as accurate as possible
        Sleep(.3);

        // Turning really specifically to the angle
        turnToAngleWhenAlreadyReallyClose(6);
    }

    // Pressing down on the counters, giving the servo time to get down
    armServo.SetDegree(95);
    Sleep(.5);

    // The "going backwards" part of foosball
    if (!hasExhaustedDeadzone)
    {
        // Physically pulling the counters over
        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .4);
        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .4);
        Sleep(1.9);

        // Stopping the motors
        leftMotor.Stop();
        rightMotor.Stop();

        // Lifting the arm off of the counters
        armServo.SetDegree(75);
        // Sleep(.5); // Put this back in if it pulls the counters too far forward again at the end

        // Moving forward a little bit
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .2);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .2);
        Sleep(1.0);

        // Stopping the motors
        leftMotor.Stop();
        rightMotor.Stop();

        // Pressing the arm onto the counters again
        armServo.SetDegree(95);
        Sleep(.5);

        // Pulling the counters back again just to be sure
        leftMotor.SetPercent(-LEFT_MOTOR_PERCENT * .2);
        rightMotor.SetPercent(-RIGHT_MOTOR_PERCENT * .2);
        Sleep(1.0);

        // Stopping the motors
        leftMotor.Stop();
        rightMotor.Stop();

        // Rotating the arm off of the motors
        armServo.SetDegree(30);
        Sleep(.5);

        // Only do this if we don't make the robot go above the dodecahedron
        leftMotor.SetPercent(LEFT_MOTOR_PERCENT * .5);
        rightMotor.SetPercent(RIGHT_MOTOR_PERCENT * .5);
        Sleep(1.0);
        leftMotor.Stop();
        rightMotor.Stop();
    }

    // TODO - Make it use goToPoint backwards to go around the top of the dodecahedron
    /* Tenative outline for what that might look like (probably better to actually go and get test RPS values for this):
    // Go above the dodecahedron
    // TODO - Add hasExhaustedDeadzone checks to these
    // TODO - Go measure the RPS there and just use goToPoint to go to like 3 different points around the edge so it's a moderately smooth turn
    goToPoint(FOOSBALL_START_X - 14, FOOSBALL_START_Y - 2, false, 0.0, false, 0.0, true, 4);

    // Go below the lever kinda
    goToPoint(FOOSBALL_START_X - 18, FOOSBALL_START_Y - 6, false, 0.0, false, 0.0, true, 4);

    // Go below the lever all the way and approximately realign
    goToPoint(FOOSBALL_START_X - 22, FOOSBALL_START_Y - 8, false, 0.0, false, 0.0, true, 4);

    // Positioning for the lever
    // More precise, slower positioning once we're nearly there
    if (!hasExhaustedDeadzone)
        goToPoint(LEVER_X, LEVER_Y, true, LEVER_HEADING, false, 1.5, false, 0);

    // Turning really precisely to the lever
    if (!hasExhaustedDeadzone)
        turnToAngleWhenAlreadyReallyClose(LEVER_HEADING);
    */

    // Going to the left part
    if (!hasExhaustedDeadzone)
        goToPoint(20, 48, false, 0.0, false, 0.0, false, 6);

    if (!hasExhaustedDeadzone)
        goToPoint(8, 48, false, 0.0, false, 0.0, false, 6);

    // Positioning for the lever
    // Approximate, faster positioning most of the way there
    if (!hasExhaustedDeadzone)
        goToPoint(LEVER_X + 2, LEVER_Y - 4, false, 0.0, false, 0.0, false, 5);

    // Positioning for the lever
    // More precise, slower positioning once we're nearly there
    if (!hasExhaustedDeadzone)
        goToPoint(LEVER_X, LEVER_Y, true, LEVER_HEADING, false, 1.5, false, 0);

    // Turning really precisely to the lever
    if (!hasExhaustedDeadzone)
        turnToAngleWhenAlreadyReallyClose(LEVER_HEADING);

    // Pressing the lever
    armServo.SetDegree(110);
    Sleep(1.0);
    armServo.SetDegree(30);

    /* It skips to right here if RPS drops */

    // Go to the top of the relevant ramp (approx.)
    goToPoint(6, 55.0, false, 0.0, false, 0.0, false, 6);

    // Running into the end button
    goToPoint(5.0, 5.0, false, 0.0, false, 0.0, false, 6);
}


