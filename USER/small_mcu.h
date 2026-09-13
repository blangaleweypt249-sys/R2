#ifndef __SMALL_MCU_H__
#define __SMALL_MCU_H__

#include "main.h"
#include "can.h"
#include "tim.h"
#include "spi.h"
#include "AS5047P.h"
#include "CAN_PUB.h"
#include "c610.h"

void Small_MCU_Init(void);
void Small_MCU_Task(void);

#endif
