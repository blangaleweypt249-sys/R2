#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include "stdint.h"
#include "WpLib.h"

#define VOFA_CHANNEL_COUNT  6U
#define VOFA_FRAME_SIZE     (VOFA_CHANNEL_COUNT * 4U + 4U)

typedef struct
{
    UART_HandleTypeDef *huart;
    float target_speed;     
    float actual_speed;     
    float target_position;  
    float actual_position;  
    float target_current;   
    float actual_current;   
}vofa_DJIdata;

extern vofa_DJIdata motor;

void Vofa_Init(UART_HandleTypeDef *huart);
void Vofa_Send(vofa_DJIdata *data);

#endif
