// ZAutoWithGPS.h

#ifndef _ZAutoWithGPS_h
#define _ZAutoWithGPS_h
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#define AWGOnGround 0
#define AWGTakeOff 1
#define AWGAutoWP 2
#define AWGLand 3
#define AWGTargetHeight 200.f//cm

static int AutoWithGPSMode = AWGOnGround;

void AutoWithGPSReset()
{
    AutoWithGPSMode = AWGOnGround;
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
static int ArrivedCounterValueWP = 0;
static int ArrivedCounterValueTakeOff = 0;
static void ArrivedCounter() //100Hz
{
    if (AutoWithGPSMode == AWGTakeOff)
    {
        if ((AWGTargetHeight - inertial_nav.get_position().z) <= 50)
            ArrivedCounterValueTakeOff++;
    }
    if (AutoWithGPSMode == AWGAutoWP)
    {
        if (wp_nav.reached_wp_destination() == true)
            ArrivedCounterValueWP++;
    }     
}

void AutoWithGPSDoTakeoff()
{
    // Set wp navigation target to safe altitude above current position
    float takeoff_alt = AWGTargetHeight;
    //takeoff_alt = max(takeoff_alt, current_loc.alt);

    // initialise wpnav destination
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z = takeoff_alt;
    wp_nav.set_wp_destination(target_pos);

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // tell motors to do a slow start
    motors.slow_start(true);


}

static void LandGpsRun()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed or landed set throttle to zero and exit immediately
    if (!ap.auto_armed || ap.land_complete) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        wp_nav.init_loiter_target();

#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif
        return;
    }

    // relax loiter target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }
    
    /* //process pilot inputs
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = g.rc_1.control_in;
            pitch_control = g.rc_2.control_in;
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }*/
    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter();

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    
    static bool land_pause = false;
    float cmb_rate = -10;//get_throttle_land();
    /*
    //pause 4 seconds before beginning land descent
    float cmb_rate;
    if (land_pause && millis() - land_start_time < 4000) {
        cmb_rate = 0;
    }
    else {
        land_pause = false;
        cmb_rate = get_throttle_land();
    }
    */

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();
}
static void LandNoGpsRun()
{
    int16_t target_roll = 0, target_pitch = 0;
    float target_yaw_rate = 0;

    // if not auto armed or landed set throttle to zero and exit immediately
    if (!ap.auto_armed || ap.land_complete) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif
        return;
    }
    /*
    // process pilot inputs
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }
*/
    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
   
    /*
    //pause 4 seconds before beginning land descent
    float cmb_rate;
    if (land_pause && millis() - land_start_time < LAND_WITH_DELAY_MS) {
        cmb_rate = 0;
    }
    else {
        land_pause = false;
        cmb_rate = get_throttle_land();
    }*/
    float cmb_rate = -10;

    // call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();
}
static void LandRun()
{
    if (GPS_ok())
        LandGpsRun();
    else
        LandNoGpsRun();
}
static void AWGAutoTakeOffRun()
{
    // if not auto armed set throttle to zero and exit immediately
    if (!ap.auto_armed) {
        // initialise wpnav targets
        wp_nav.shift_wp_origin_to_current_pos();
        // reset attitude control targets
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        // tell motors to do a slow start
        motors.slow_start(true);
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.set_speed_z(-10.f, 10.f);
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}
void AutoWithGPS()
{
    switch (AutoWithGPSMode)
    {
    case AWGOnGround:
        AutoWithGPSDoTakeoff();
        AutoWithGPSMode = AWGTakeOff;
        ArrivedCounterValueTakeOff = 0;
        break;
    case AWGTakeOff:
        AWGAutoTakeOffRun();
        if (ArrivedCounterValueTakeOff >= 100)
        {
            AutoWithGPSMode = AWGAutoWP;
            Vector3f xyzTarget;
            xyzTarget.operator()(500, 0,AWGTargetHeight);
            //向东飞行50m
            AutoWpStart(xyzTarget);
            ArrivedCounterValueWP = 0;
        }
        break;
    case AWGAutoWP:
        auto_wp_run();
        if (ArrivedCounterValueWP >= 100)
        {
            AutoWithGPSMode = LAND;
        }
        break;
    case LAND:
    default:
        //land_run();//set_speed_z(float speed_down, float speed_up)
        LandRun();
    }    
}
#endif
