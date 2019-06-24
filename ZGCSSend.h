//ZGCSSend.h
#ifndef _ZGCS_h
#define _ZGCS_h
#include "GCS.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
void GcsSendHeartBeat()//心跳包的发送，10Hz
{
    gcs_send_heartbeat();
    
}
void GetPacket()
{
    gcs_check_input();
}
bool SendLongCommand(mavlink_channel_t chan, mavlink_command_long_t packet)
{
    uint16_t txspace = comm_get_txspace(chan);

    if (telemetry_delayed(chan)) {
        return false;
    }

#if HIL_MODE != HIL_MODE_SENSORS
    // if we don't have at least 250 micros remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (scheduler.time_available_usec() < 250 && motors.armed()) {
        //gcs_out_of_time = true;
        return false;
    }
#endif
    packet.command = (uint16_t)0;
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_LONG, (const char *)&packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN, MAVLINK_MSG_ID_COMMAND_LONG_CRC);
}



//gcs().handleMessage(mavlink_message_t* msg);
#endif
