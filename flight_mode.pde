/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if STABILIZERE>=0
#include "ZStabilizeRe.h"
#endif
/*
 * flight.pde - high level calls to set and update flight modes
 *      logic for individual flight modes is in control_acro.pde, control_stabilize.pde, etc
 */

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was succesfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
static bool set_mode(uint8_t mode)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !motors.armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        return true;
    }

    switch(mode) {
        case ACRO:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_acro_init(ignore_checks);
            #else
                success = acro_init(ignore_checks);
            #endif
            break;

        case STABILIZE:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_stabilize_init(ignore_checks);
            #else
                success = stabilize_init(ignore_checks);
            #endif
            break;

        case ALT_HOLD:
            success = althold_init(ignore_checks);
            break;

        case AUTO:
            success = auto_init(ignore_checks);
            break;

        case CIRCLE:
            success = circle_init(ignore_checks);
            break;

        case LOITER:
            success = loiter_init(ignore_checks);
            break;

        case GUIDED:
            success = guided_init(ignore_checks);
            break;

        case LAND:
            success = land_init(ignore_checks);
            break;

        case RTL:
            success = rtl_init(ignore_checks);
            break;

#if OPTFLOW == ENABLED
        case OF_LOITER:
            success = ofloiter_init(ignore_checks);
            break;
#endif

        case DRIFT:
            success = drift_init(ignore_checks);
            break;

        case SPORT:
            success = sport_init(ignore_checks);
            break;

        case FLIP:
            success = flip_init(ignore_checks);
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            success = autotune_init(ignore_checks);
            break;
#endif

#if POSHOLD_ENABLED == ENABLED
        case POSHOLD:
            success = poshold_init(ignore_checks);
            break;
#endif

        default:
            success = false;
            break;
    }

    // update flight mode
    if (success) {
        // perform any cleanup required by previous flight mode
        exit_mode(control_mode, mode);
        control_mode = mode;
        Log_Write_Mode(control_mode);

#if AC_FENCE == ENABLED
        // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
        // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
        // but it should be harmless to disable the fence temporarily in these situations as well
        fence.manual_recovery_start();
#endif
    }else{
        // Log error that we failed to enter desired flight mode
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
    }

    // return success or failure
    return success;
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more

static void update_flight_mode()
{
   // switch (control_mode) {
   switch(NewControlModeSwitch)
   {
#if TEST>=0
   case TEST:
       ZTest();
       break;
#endif // TEST>=0


#if AUTOWITHGPS>=0
   case AUTOWITHGPS:
       AutoWithGPS();
       break;
#endif

#if STABILIZERE>=0
   case STABILIZERE:
       StabilizeRe();
       break;
#endif

#if STABILIZE>=0
    case STABILIZE://飞控控制无人机保持稳定，同时接收遥控器数据。可用于起飞降落
#if FRAME_CONFIG == HELI_FRAME
        heli_stabilize_run();
#else
        stabilize_run();
#endif
        break;
#endif // STABILIZE>=0

#if AUTO1>=0
    case AUTO1://测试程序1，起飞1m，悬停60s，降落        
        autorun1();
        break;
#endif // AUTO1>=0
        
#if AUTO2>=0
    case AUTO2://测试程序2，起飞，向东飞行50m，降落
        autorun2();
        break;
#endif // AUTO2>=0

#if RESET>=0
    case RESET://重启测试程序中会用到的参数
        reset();
        break;
#endif // RESET>=0

#if TEST1>=0
    case TEST1:
        land_run();
        break;
#endif // Test1>=0

#if TEST2>=0
    case TEST2:
        Test2();
        break;
#endif // Test2>=0

#if ALT_HOLD>0
    case ALT_HOLD://定高模式：除非大幅度更改遥控器油门值，否则无人机保持现有油门量不变
        althold_run();
        break;
#endif
#if AUTO>0
    case AUTO://在GPS引导下，按预定任务进行飞行
        auto_run();
        break;
#endif // AUTO>0
#if LAND>0
    case LAND://降落
        //AutoWithGPSReset();
        StabilizeReReset();
        land_run();
        break;
#endif // LAND>0
#if CIRCLE>0
    case CIRCLE://以当前位置为圆心，机头指向圆心进行绕圈
        circle_run();
        break;
#endif

        /*
        case ACRO://全手动控制
                        
            #if FRAME_CONFIG == HELI_FRAME
                heli_acro_run();
            #else
                acro_run();
            #endif
            break;
            */



            /*

            

           
            

      
            

        case LOITER://GPS（保持平面位置稳定）+气压计（保持高度稳定）
            loiter_run();
            break;
          

        case GUIDED://与地面站建立联系后，由地面站数据进行控制
            guided_run();
            break;
            



            
        case RTL://GPS引导下自动返航<-返航位置见控制模式详解
            rtl_run();
            break;
           

#if OPTFLOW == ENABLED
        case OF_LOITER:
            ofloiter_run();
            break;
#endif

        case DRIFT://漂移
            drift_run();
            break;

        case SPORT://运动
            sport_run();
            break;

        case FLIP://翻转
            flip_run();
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE://自动调参
            autotune_run();
            break;
#endif

#if POSHOLD_ENABLED == ENABLED
        case POSHOLD://手控定点
            poshold_run();
            break;
#endif
*/

    }
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
static void exit_mode(uint8_t old_control_mode, uint8_t new_control_mode)
{
#if AUTOTUNE_ENABLED == ENABLED
    if (old_control_mode == AUTOTUNE) {
        autotune_stop();
    }
#endif

    // stop mission when we leave auto mode
    if (old_control_mode == AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if MOUNT == ENABLED
        camera_mount.set_mode_to_default();
#endif  // MOUNT == ENABLED
    }

    // smooth throttle transition when switching from manual to automatic flight modes
    if (manual_flight_mode(old_control_mode) && !manual_flight_mode(new_control_mode) && motors.armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
    }
    
#if FRAME_CONFIG == HELI_FRAME
    // firmly reset the flybar passthrough to false when exiting acro mode.
    if (old_control_mode == ACRO) {
        attitude_control.use_flybar_passthrough(false);
    }
#endif //HELI_FRAME
}

// returns true or false whether mode requires GPS
static bool mode_requires_GPS(uint8_t mode) {
    switch(mode) {
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case DRIFT:
        case POSHOLD:
            return true;
        default:
            return false;
    }

    return false;
}

// manual_flight_mode - returns true if flight mode is completely manual (i.e. roll, pitch, yaw and throttle are controlled by pilot)
static bool manual_flight_mode(uint8_t mode) {
    switch(mode) {
        case ACRO:
        case STABILIZE:
            return true;
        default:
            return false;
    }

    return false;
}

// mode_allows_arming - returns true if vehicle can be armed in the specified mode
//  arming_from_gcs should be set to true if the arming request comes from the ground station
static bool mode_allows_arming(uint8_t mode, bool arming_from_gcs) {
    if (manual_flight_mode(mode) || mode == LOITER || mode == ALT_HOLD || mode == POSHOLD || (arming_from_gcs && mode == GUIDED)) {
        return true;
    }
    return false;
}

//
// print_flight_mode - prints flight mode to serial port.
//
static void
print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case STABILIZE:
        port->print_P(PSTR("STABILIZE"));
        break;
    case ACRO:
        port->print_P(PSTR("ACRO"));
        break;
    case ALT_HOLD:
        port->print_P(PSTR("ALT_HOLD"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case GUIDED:
        port->print_P(PSTR("GUIDED"));
        break;
    case LOITER:
        port->print_P(PSTR("LOITER"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    case CIRCLE:
        port->print_P(PSTR("CIRCLE"));
        break;
    case LAND:
        port->print_P(PSTR("LAND"));
        break;
    case OF_LOITER:
        port->print_P(PSTR("OF_LOITER"));
        break;
    case DRIFT:
        port->print_P(PSTR("DRIFT"));
        break;
    case SPORT:
        port->print_P(PSTR("SPORT"));
        break;
    case FLIP:
        port->print_P(PSTR("FLIP"));
        break;
    case AUTOTUNE:
        port->print_P(PSTR("AUTOTUNE"));
        break;
    case POSHOLD:
        port->print_P(PSTR("POSHOLD"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}

