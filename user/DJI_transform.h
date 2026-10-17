#ifndef __DJI_transform_H__
#define __DJI_transform_H__

#include <stdint.h>
#include <stdbool.h>
#include "vesc.h" 

typedef enum
{
  DJI_STOP = 0,   //停止(电流为0)
  DJI_SPEED,      //速度模式
  DJI_POSITION,   //位置模式
  DJI_to_zero,    //回零(转偏置角)
  DJI_sign_zero   //标零
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

  float angle_d;
  float compensation_delta;
  float speed_cosk;
}DJI_HlemMotor;

typedef union
{
  float value;
  uint8_t data[4];
} packet;


void DJI2006_Init(DJI_HlemMotor *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void DJI2006_speed(DJI_HlemMotor *motor,float speed);
void DJI2006_position(DJI_HlemMotor *motor,float position);
void DJI2006_stop(DJI_HlemMotor *motor);
void DJI2006_to_zero(DJI_HlemMotor *motor);
void DJI2006_sign_zero(DJI_HlemMotor *motor);
void DJI2006_Analysisdata(DJI_HlemMotor *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);

#endif
