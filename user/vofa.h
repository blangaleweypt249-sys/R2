#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include "stdint.h"
#include "Dji_Master.h"

#define VOFA_CHANNEL_COUNT  4U
#define VOFA_FRAME_SIZE     (VOFA_CHANNEL_COUNT * 4U + 4U)

void Vofa_Init(UART_HandleTypeDef *huart);

/*
 *   通道0 = target_speed    目标转速(rpm)
 *   通道1 = speed_rpm       实际转速(rpm)
 *   通道2 = target_position 目标角度(deg)
 *   通道3 = angle_deg       实际角度(deg)
 */
void Vofa_Send(const DJI_Motor *motor);

#endif
