#ifndef __DJI_transform_H__
#define __DJI_transform_H__

#include <stdint.h>
#include <stdbool.h>
#include "vesc.h"
#include "WpLib.h" 

typedef enum
{
  Helm_STOP = 0,   //停止(电流为0)
  Helm_SPEED,      //速度模式
  Helm_POSITION,   //位置模式
  Helm_to_zero,    //回零(转偏置角)
  Helm_sign_zero   //标零
}Helm_Mode;

typedef struct
{
  FDCAN_HandleTypeDef *hfdcan;
  uint32_t can_id;
  uint8_t motor_id;
  Helm_Mode mode;
  float target_speed;
  float target_position;

  float actual_angle;
  bool calibration_valid;

  float angle_d;
  float compensation_delta;
  float speed_cosk;
  int num[3];
}DJI_HlemMotor;

void Helm2006_Init(DJI_HlemMotor *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void Helm2006_speed(DJI_HlemMotor *motor,float speed);
void Helm2006_position(DJI_HlemMotor *motor,float position);
void Helm2006_stop(DJI_HlemMotor *motor);
void Helm2006_to_zero(DJI_HlemMotor *motor);
void Helm2006_sign_zero(DJI_HlemMotor *motor);
void Helm2006_Analysisdata(DJI_HlemMotor *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);

#endif
