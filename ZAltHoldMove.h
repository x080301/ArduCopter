// ZAltHoldMove.h

#ifndef _ZALTHOLDMOVE_H
#define _ZALTHOLDMOVE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif



void DoMove(Vector3f destination)
{
    // initialise wpnav
    wp_nav.set_wp_destination(destination);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}
void AltholdMoveRun()
{
    // if not auto armed set throttle to zero and exit immediately
    if (!ap.auto_armed) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
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
        if (target_yaw_rate != 0) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }
    else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(), true);
    }
}
/*
平飞。
xy速度定义为10cm/s，上下速度定义为20cm/s
mode定义：0静止，1左右平飞，2前后平飞
方向定义：以从上向下看，机头朝前，为例
+  0   -
x   右   -   左
y   前   -   后
*/
void ZAltHoldMove(int mode,int direction)
{
    //计算目标GPS点
    switch (mode)
    {
    case 1:
        Vector3f destination = inertial_nav.get_position();
        destination.x += (direction > 0 ? 1 : -1) * 5000;
        DoMove(destination);
        break;
    case 2:
        Vector3f destination = inertial_nav.get_position();
        destination.y += (direction > 0 ? 1 : -1) * 5000;
        DoMove(destination);
        break;
    case 0:
    default:
        Vector3f DoMove(inertial_nav.get_position());
        break;
    }
    //执行飞行
    AltholdMoveRun();
}

#endif