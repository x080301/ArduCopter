#ifndef _ZGCSSend_h
#define _ZGCSSend_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
void GcsSendHeartBeat()//心跳包，1Hz
{
    gcs_send_heartbeat();
}
#endif
