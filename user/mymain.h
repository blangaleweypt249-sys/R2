#ifndef __MYMAIN_H__
#define __MYMAIN_H__

#include "main.h"
#include <stdbool.h>
#include "fdcan.h"
#include "usart.h"
#include "vesc.h"
#include "DJI_2006.h"
#include "helm_chassis.h"
#include "imu_100A.h"
#include "dt35.h"
#include "pid.h"
#include "cmsis_os2.h"
#include "math.h"

typedef union
{
    uint16_t u16[4];
    uint8_t u8[8];
}U8_to_U16;

typedef struct 
{
  U8_to_U16 rocker;
  U8_to_U16 shoulder;
}HandleData;

extern HandleData handle_data;
extern VESC vesc_motor[4];
extern DJI_Motor dji_motor[4];
extern uint8_t receive_handle[19];
extern uint8_t receive_imu[7];
extern uint8_t receive_dt35[19];
extern osMessageQueueId_t handleQueueHandle;
extern osMessageQueueId_t imuQueueHandle;
void My_init(void);
void Handle_Analysis(uint8_t *data);

#endif
