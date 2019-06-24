// ZTest.h

#ifndef _ZTEST_H
#define _ZTEST_H

#include "ZZcomtrol.h"
#include "ZAltHoldMove.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

static int TestMode = 0;
static int TestTime = 0;
static void TestTimeHandle()
{
    TestTime += 1;
}
void ZTest()
{
    switch (TestMode)
    {
    case 0://在地面
        ZZcomtrol(4);
        TestMode = 1;
        break;
    case 1://起飞
        ZZcomtrol(1);
        if (inertial_nav.get_position().z > 300)
        {
            TestMode = 2;
            TestTime = 0;
        }
        break;
    case 2://悬停5s
        ZZcomtrol(0);
        if (TestTime >= 50)
        {
            TestMode = 3;
            TestTime = 0;
        }
    case 3://平飞左5s
        ZAltHoldMove(1, -1);
        if (TestTime >= 50)
        {
            TestMode = 4;
            TestTime = 0;
        }
    case 4://平飞右5s
        ZAltHoldMove(1, 1);
        if (TestTime >= 50)
        {
            TestMode = 5;
            TestTime = 0;
        }
    case 5://平飞前5s
        ZAltHoldMove(2, 1);
        if (TestTime >= 50)
        {
            TestMode = 6;
            TestTime = 0;
        }
    case 6://平飞后5s
        ZAltHoldMove(2,- 1);
        if (TestTime >= 50)
        {
            TestMode = 7;
            TestTime = 0;
        }
    case 7://下降1s
        ZZcomtrol(3);
        if (TestTime >= 10)
        {
            TestMode = 8;
            TestTime = 0;
        }
    case 8://悬停5s
        ZZcomtrol(0);
        if (TestTime >= 50)
        {
            TestMode = 9;
            TestTime = 0;
        }
    case 9://降落
        ZZcomtrol(3);
    }
}
void ZTestReset()
{
    TestMode = 0;
    TestTime = 0;
}

#endif

