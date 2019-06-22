#ifndef _ZGCS_h
#define _ZGCS_h
#include "GCS.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
void GcsSendHeartBeat()//心跳包的发送，1Hz
{
    gcs_send_heartbeat();
    
}
void GetPacket()
{
    gcs_check_input();
}


//try_send_message(enum ap_message id)

//gcs().handleMessage(mavlink_message_t* msg);
#endif
