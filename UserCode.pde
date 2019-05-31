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

#if TEST2>=0
static int TestTime = 0;
static int TestFlag = 0;
void Test2()
{
    if (TestFlag == 0)
        stabilize_run();
    else
        land_run();
}
#endif // Test2>=0



#if AUTO1>=0
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

#if AUTO2>=0
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

#if RESET>=0
void reset()//重设各初始测试量
{
    processflag = 0;
    AltHoldTime = 0;
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
    ReadControlSwitch();
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