#ifndef __MYMAIN_H__
#define __MYMAIN_H__

#include "main.h"
#include <stdbool.h>
#include "fdcan.h"
#include "usart.h"
#include "vesc.h"
#include "DJI_2006.h"
#include "helm_chassis.h"
#include "pid.h"
#include "cmsis_os2.h"

extern VESC vesc_motor[4];
extern osMessageQueueId_t handleQueueHandle;
extern osMessageQueueId_t imuQueueHandle;
void My_init(void);

#endif
