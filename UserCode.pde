/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
static int processflag = 0;
static int AltHoldTime = 0;

void takeoff(int TargetAlt)
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
    pos_control.set_alt_target(TargetAlt+50);
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    float Z_position = inertial_nav.get_altitude();//检测高度以确定什么时候进入降落模块
    if (Z_position > TargetAlt)
    {
        processflag = 1;
    }
}
void AutoWpRun(Vector3f xyzTarget)
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

    // run waypoint controller//设定目标位置
    pos_control.set_pos_target(xyzTarget);
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

    //确认是否可以进入降落阶段
    if (pos_control.get_distance_to_target() < 100)
    {
        processflag = 2;
    }
}

#if AUTO1!=-1
void autorun1()//标签：我们的代码可以在这儿，以一个mode的形式，由遥控器选定后执行
{
    if (processflag == 0)
    {
        //起飞
        takeoff(100);
    }
    else if (processflag == 1)
    {
        althold_run();
        //悬停
    }
    else
    {
        //降落
        land_run();
    }
}
#endif

#if AUTO2!=-1
void autorun2()
{
    if (processflag == 0)
    {
        //起飞
        takeoff(500);
    }
    else if (processflag == 1)
    {    

        Vector3f xyzTarget;
        xyzTarget.operator()(5000, 0, 500);
        AutoWpRun(xyzTarget);
        //向东飞行50m
    }
    else
    {
        //降落
        land_run();
    }
}
#endif

#if RESET!=-1
void reset()//重设各初始测试量
{
    processflag = 0;
    AltHoldTime = 0;
}
#endif

#if Stabilize_1!=-1
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
    if (z_SpeedControl < THROTTLE_IN_MIDDLE-DeedBand) {
        // below the deadband
        SpeedOut = -((float)(450 - z_SpeedControl)) / 4.5f;
    }
    else if (z_SpeedControl > THROTTLE_IN_MIDDLE+DeedBand) {
        // above the deadband
        SpeedOut = ((float)(z_SpeedControl - 550)) / 4.5f;
    }
    else {
        // must be in the deadband
        SpeedOut = 0.0f;
    }

    return SpeedOut;
}
void Stabilize()
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

    // 获取由3号通道传入的值，以控制升降及升降速度
    float ZSpeed = get_pilot_desired_Z_Speed(g.rc_3.control_in);
    pos_control.set_speed_z(fabs(ZSpeed), fabs(ZSpeed));
    if (ZSpeed > 0)
        pos_control.set_alt_target(inertial_nav.get_altitude() + 200);
    else
        pos_control.set_alt_target(inertial_nav.get_altitude() - 200);
    pos_control.update_z_controller();

    // call attitude controller         一番处理后这里在输出姿态控制值，水平面内的移动也在这里实现
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
    // body-frame rate controller is run directly from 100hz loop

}
#endif

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

#define USERHOOK_SUPERSLOWLOOP
#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    if (AltHoldTime++ > 60)
    {
        processflag = 2;
    }
}
#endif