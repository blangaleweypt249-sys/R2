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
  float target_speed;
  float target_position;

  float actual_angle;
  bool calibration_valid;

}DJI_Motor;

typedef union
{
  float value;
  uint8_t data[4];
} packet;


void DJI2006_Init(DJI_Motor *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void DJI2006_speed(DJI_Motor *motor,float speed);
void DJI2006_position(DJI_Motor *motor,float position);
void DJI2006_stop(DJI_Motor *motor);
void DJI2006_to_zero(DJI_Motor *motor);
void DJI2006_sign_zero(DJI_Motor *motor);
void DJI2006_Analysisdata(DJI_Motor *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);

#endif
