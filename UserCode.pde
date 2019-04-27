/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
static uflag

void takeoff()
{
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

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // run waypoint controller//here wpnav update it's alt target 
    wp_nav.update_wpnav();

    // call z-axis position controller//设置目标高度，调用zcontroller
    pos_control.set_alt_target(50);
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

void land()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if (!ap.auto_armed || ap.land_complete) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        // set target to current position
        wp_nav.init_loiter_target();
        return;
    }

    // relax loiter targets if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // process pilot's input
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
    }

    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter();

    // call z-axis position controller
    pos_control.set_alt_target_from_climb_rate(get_throttle_land(), G_Dt, true);
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

void autorun()//标签：我们的代码可以在这儿，以一个mode的形式，由遥控器选定后执行
{
    //起飞
    takeoff();
 

    //降落
    land();
}

#ifdef USERHOOK_INIT//标签：我们的代码也可以在这儿及以下的部分，会以一定的频率反复执行
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif