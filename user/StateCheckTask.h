#ifndef __StateCheckTask_H__
#define __StateCheckTask_H__

#include "main.h"
#include "cmsis_os2.h"
#include "mymain.h"

#define STATE_TIMEOUT_COUNT    20  //大概200ms更新一次


typedef struct
{
    bool handle;
    bool imu;
    bool dt35[4];
    bool dji_motor[4];
    bool vesc_motor[4];
} Device_Offline_State_t;

extern Device_Offline_State_t Device_Offline_State;

bool Check_State(int *num, int timeout);

#endif
