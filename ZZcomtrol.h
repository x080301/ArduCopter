// ZZcomtrol.h

#ifndef _ZZCONTROL_H
#define _ZZCONTROL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#define UPSPEED 20.f
#define DOWNSPEED -10.f
void DoUp_Xcm(int alt)
{
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z += alt;
    wp_nav.set_wp_destination(target_pos);

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}
void DoTakeoOff()
{
    DoUp_Xcm(400);
    // tell motors to do a slow start
    motors.slow_start(true);
}
//true:Up,false:Hold
static void DoUpHoldControler(bool UpHold)
{
    Vector3f target_pos = inertial_nav.get_position();
    if(UpHold)
        target_pos.z += 300;
    wp_nav.set_wp_destination(target_pos);

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}
void UpOrHoldRun()
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
    pos_control.set_speed_z(DOWNSPEED, UPSPEED);
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}
void Down()
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
    float cmb_rate = DOWNSPEED;

    // call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();
}
/*
升降控制
0,1,2,3,4
定高、起飞、升、降、在地面
*/
void ZZcomtrol(int mode)
{
    switch (mode)
    {
    case 4:
        DoTakeoOff();
        break;
    case 1:
        UpOrHoldRun();
        break;
    case 2:
        DoUpHoldControler(true);
        UpOrHoldRun();
        break;
    case 3:
        Down();
        break;
    case 0:
    default:
        DoUpHoldControler(false);
        UpOrHoldRun();
        break;
    }
}


#endif

