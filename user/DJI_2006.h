#ifndef __DJI_2006_H__
#define __DJI_2006_H__

#include <stdint.h>
#include <stdbool.h>
#include "vesc.h" 

typedef enum
{
  DJI_STOP = 0,
  DJI_SPEED,
  DJI_POSITION,
  DJI_to_zero,
  DJI_sign_zero
}DJI_Mode;

typedef struct
{
  FDCAN_HandleTypeDef *hfdcan;
  uint32_t can_id;
  uint8_t motor_id;
  DJI_Mode mode;
  float ftarget_speed;
  float ftarget_position;
  int16_t target_speed;
  int16_t target_position;

  //反馈
  //int16_t relative_position;
  //uint16_t actual_position;
  //int16_t actual_speed;
  float actual_angle;
  //bool dji_state;
  //uint8_t dji_feed_id;
}DJI_Motor;

void DJI2006_Init(DJI_Motor *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void DJI2006_speed(DJI_Motor *motor,float speed);
void DJI2006_position(DJI_Motor *motor,float position);
void DJI2006_stop(DJI_Motor *motor);
void DJI2006_to_zero(DJI_Motor *motor);
void DJI2006_sign_zero(DJI_Motor *motor);
void DJI2006_Analysisdata(DJI_Motor *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);

#endif
