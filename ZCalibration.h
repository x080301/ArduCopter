// ZCalibration.h

#ifndef _ZCalibration_h
#define _ZCalibration_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
void CalibrationRc3MaxMin()//100Hz
{
    static int loopi = 0;
    static int value = 0;
    value += g.rc_3.control_in;
    loopi++;
    if (loopi == 10)
    {
        value = value / loopi;
        if (value > Rc3Max)
            Rc3Max = value;
        if (value < Rc3Min)
            Rc3Min = value;
        loopi = 0;
        value = 0;
    }        
}



#endif


