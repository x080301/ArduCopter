// ZAutoWithGPS.h

#ifndef _ZAutoWithGPS_h
#define _ZAutoWithGPS_h
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#define OnGround 0
#define TakeOff 1
#define AutoWP 2
#define Land 3
static int AutoWithGPSMode = OnGround;

void AutoWithGPSReset()
{
    AutoWithGPSMode = OnGround;
}

static void AutoWpStart(const Vector3f& destination)
{
    auto_mode = Auto_WP;
    // initialise wpnav
    wp_nav.set_wp_destination(destination);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

#define ARRIVEDCOUNTER
static int ArrivedCounterValue = 0;
static void ArrivedCounter() //100Hz
{
    if (wp_nav.reached_wp_destination() == 1)
        ArrivedCounterValue++;
}

void AutoWithGPSDoTakeoff(float TakeoffAlt)
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

void AutoWithiGPS()
{
    switch (AutoWithGPSMode)
    {
    case OnGround:
        AutoWithGPSDoTakeoff(500);
        AutoWithGPSMode = TakeOff;
        break;
    case TakeOff:
        auto_takeoff_run();
        if (current_loc.alt >= 500)
        {
            AutoWithGPSMode = AutoWP;
            Vector3f xyzTarget;
            xyzTarget.operator()(2000, 0, 500);
            //向东飞行20m
            AutoWpStart(xyzTarget);
        }
        break;
    case AutoWP:
        auto_wp_run();
        if (ArrivedCounterValue >= 100)
        {
            AutoWithGPSMode = LAND;
            ArrivedCounterValue = 0;
        }
        break;
    case Land:
    default:
        land_run();
    }



    
}
#endif
