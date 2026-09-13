#ifndef __C610_H__
#define __C610_H__

#include "main.h"
#include "can.h"
#include "PID.h"

//选择小板
#define BOARD_NODE_3

#if defined(BOARD_NODE_1)

#define SMALL_MCU_NODE_ID 1
#define STEER_ZERO_ENCODER 0
#define STEER_OFFSET_DEG 45.0f
#elif defined(BOARD_NODE_2)
#define SMALL_MCU_NODE_ID 2
#define STEER_ZERO_ENCODER 0
#define STEER_OFFSET_DEG 45.0f
#elif defined(BOARD_NODE_3)
#define SMALL_MCU_NODE_ID 3
#define STEER_ZERO_ENCODER 0
#define STEER_OFFSET_DEG 0.0f
#elif defined(BOARD_NODE_4)
#define SMALL_MCU_NODE_ID 4
#define STEER_ZERO_ENCODER 0
#define STEER_OFFSET_DEG 45.0f

#endif

#define C610_MOTOR_NUM       1//自定挂载电机数量

#define C610_MODE_STOP       0
#define C610_MODE_SPEED      1
#define C610_MODE_Position   2
#define C610_MODE_HOME       3
#define C610_MODE_CAPTURE_ZERO 4

typedef struct 
{ 
    int16_t speed_rpm; 
    float angle_deg; 
    int16_t given_current; 
    uint8_t temperature; 
    uint8_t is_online; 
    uint8_t mode; 
    float target; 
    
    PID_Controller speed_pid; 
    PID_Controller position_pid; 

    uint16_t last_encoder; 
    int32_t total_encoder; 
    uint32_t last_feedback_tick; 
    uint8_t first_feedback; 
    uint16_t zero_encoder;
    int8_t direction;
    int16_t output_current; 
} C610_Motor_t;

typedef struct 
{ 
    CAN_HandleTypeDef *can_handle; 
    volatile uint8_t home_active; 
    C610_Motor_t motor[C610_MOTOR_NUM]; 
} C610_Control_t;

extern C610_Control_t C610;

void C610_Init(CAN_HandleTypeDef *hcan);
void C610_Receive(uint32_t can_id, uint8_t *data);
void C610_Set_Command(uint8_t motor_id, uint8_t mode, int16_t target);
void C610_Start_Home_All(void);
void C610_Control(void);
void C610_Stop_All(void);

#endif
