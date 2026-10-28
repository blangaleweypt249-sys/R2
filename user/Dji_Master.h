#ifndef __DJI_MASTER_H__
#define __DJI_MASTER_H__

#include "stdint.h"
#include "fdcan.h"
#include "bsp_can.h"
#include "pid.h"

typedef struct
{
    FDCAN_HandleTypeDef *hcan;
    uint8_t motor_id;
    uint16_t feedback_id;
    float target_speed;
    float target_position;
    int16_t target_current;
    PID speed_pid;
    PID position_pid;

    //反馈值
    uint16_t encoder;
    uint16_t last_encoder;
    int32_t total_encoder;
    float speed_rpm; 
    int16_t torque_current;

    int16_t output_current;
    float angle_deg;
    float zero_offset;
    uint32_t last_command_tick;
    uint8_t first_feedback;
    uint8_t online;
    int num[3];
} DJI_Motor;


extern DJI_Motor dji_3508[4];
extern DJI_Motor dji_2006[4];

void DJI_2006_Init(DJI_Motor *motor, FDCAN_HandleTypeDef *hcan, uint8_t motor_id);
void DJI_2006CAN_Trans_1(FDCAN_HandleTypeDef *hcan);
void DJI_2006CAN_Trans_2(FDCAN_HandleTypeDef *hcan);
void DJI_2006SetCurrent(DJI_Motor *motor, int16_t current);
void DJI_2006SetSpeed(DJI_Motor *motor, float speed_rpm);
void DJI_2006SetPosition(DJI_Motor *motor, float position_deg);
void DJI_2006ReadDate(DJI_Motor *motor, uint8_t rx_data[8],
                      FDCAN_RxHeaderTypeDef *rx_header);

void DJI_3508_Init(DJI_Motor *motor, FDCAN_HandleTypeDef *hcan, uint8_t motor_id);
void DJI_3508CAN_Trans_1(FDCAN_HandleTypeDef *hcan);
void DJI_3508CAN_Trans_2(FDCAN_HandleTypeDef *hcan);
void DJI_3508SetCurrent(DJI_Motor *motor, int16_t current);
void DJI_3508SetSpeed(DJI_Motor *motor, float speed_rpm);
void DJI_3508SetPosition(DJI_Motor *motor, float position_deg);
void DJI_3508ReadDate(DJI_Motor *motor, uint8_t rx_data[8],
                      FDCAN_RxHeaderTypeDef *rx_header);

#endif
