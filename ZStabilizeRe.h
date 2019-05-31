// ZStabilizeRe.h

#ifndef _ZStabilizeRe_h
#define _ZStabilizeRe_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

float get_pilot_desired_Z_Speed(int16_t z_SpeedControl)
{
    float SpeedOut;

    // exit immediately in the simple cases
    if (z_SpeedControl == 0 || g.throttle_mid == 500) {
        return z_SpeedControl;
    }

    // ensure reasonable throttle values
    z_SpeedControl = constrain_int16(z_SpeedControl, 0, 1000);
    g.throttle_mid = constrain_int16(g.throttle_mid, 300, 700);

    // check throttle is above, below or in the deadband
    int DeedBand = 50;
    if (z_SpeedControl < THROTTLE_IN_MIDDLE - DeedBand) {
        // below the deadband
        SpeedOut = -((float)(450 - z_SpeedControl)) / 4.5f;
    }
    else if (z_SpeedControl > THROTTLE_IN_MIDDLE + DeedBand) {
        // above the deadband
        SpeedOut = ((float)(z_SpeedControl - 550)) / 4.5f;
    }
    else {
        // must be in the deadband
        SpeedOut = 0.0f;
    }

    return SpeedOut;
}
void StabilizeRe()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if (!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();//simple_mode 即机头方向不变的模式

                         // convert pilot input to lean angles
                         // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);


    /*已被改写
    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // call attitude controller         一番处理后这里在输出姿态控制值，水平面内的移动也在这里实现
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle          在这里输出油门值，高度控制在这里实现
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);
    */



    //若电机未启动
    // if not auto armed, set throttle to zero and exit immediately
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

    // 获取由3号通道传入的值，以控制升降及升降速度
    float ZSpeed = get_pilot_desired_Z_Speed(g.rc_3.control_in);
    pos_control.set_speed_z(fabs(ZSpeed), fabs(ZSpeed));
    if (ZSpeed > 0)
        pos_control.set_alt_target(inertial_nav.get_altitude() + 200);
    else if (ZSpeed < 0)
        pos_control.set_alt_target(inertial_nav.get_altitude() - 200);
    else
        pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.update_z_controller();

    // call attitude controller         一番处理后这里在输出姿态控制值，水平面内的移动也在这里实现
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
    // body-frame rate controller is run directly from 100hz loop
}

#endif

