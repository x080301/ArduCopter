// ZStabilizeRe.h

#ifndef _ZStabilizeRe_h
#define _ZStabilizeRe_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#define OnGround 0
#define TakeOff 1
#define AltHold 2
static int StabilizeReMode = OnGround;

void StabilizeReReset()
{
    StabilizeReMode = OnGround;
}

void StabilizeReDoTakeoff(float TakeoffAlt)
{
    // Set wp navigation target to safe altitude above current position
    float takeoff_alt = TakeoffAlt;
    takeoff_alt = max(takeoff_alt, current_loc.alt);

    // initialise wpnav destination
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z = takeoff_alt;
    wp_nav.set_wp_destination(target_pos);

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // tell motors to do a slow start
    motors.slow_start(true);
}

void StabilizeRe()
{
    switch (StabilizeReMode)
    {
    case OnGround:
        StabilizeReDoTakeoff(300);
        StabilizeReMode = TakeOff;
        break;
    case TakeOff:
        auto_takeoff_run();
        if (current_loc.alt >= 300)
            StabilizeReMode = AltHold;
        break;
    case AltHold:
        althold_run();
        break;
    default:
        break;
    }
}
#endif

