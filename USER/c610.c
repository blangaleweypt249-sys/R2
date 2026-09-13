#include "c610.h"
#include "PID.h"

#define C610_FEEDBACK_ID_1       0x201U//反馈ID起始值
#define C610_CURRENT_ID_1_4      0x200U//1-4号电机控制ID
#define C610_CURRENT_ID_5_8      0x1FFU//5-8号电机控制ID
#define C610_ENCODER_RANGE       8192//一圈计数个数
#define C610_ENCODER_HALF        4096
#define C610_GEAR_RATIO          57.8f//减速比
#define C610_MAX_CURRENT         10000.0f
#define C610_MAX_POSITION_RPM    3000.0f
#define C610_FEEDBACK_TIMEOUT_MS 50
#define C610_HOME_TARGET_DEG     0.0f
#define C610_HOME_FINISH_DEG     0.5f
#define C610_HOME_FINISH_RPM     5

C610_Control_t C610;

static CAN_HandleTypeDef *C610_CAN;

static void C610_Send_Group(uint32_t can_id, uint8_t first_motor)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8] = {0};
    uint32_t mailbox = 0;
    uint8_t i;
    uint8_t motor;

    if (C610_CAN == 0)
    {
        return;
    }

    tx_header.StdId = can_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    for (i = 0; i < 4; i++)
    {
        motor = (uint8_t)(first_motor + i);
        if (motor < C610_MOTOR_NUM)
        {
            tx_data[2U * i] = (uint8_t)(C610.motor[motor].output_current >> 8);
            tx_data[2U * i + 1] = (uint8_t)C610.motor[motor].output_current;
        }
    }

    HAL_CAN_AddTxMessage(C610_CAN, &tx_header, tx_data, &mailbox);
}

void C610_Init(CAN_HandleTypeDef *hcan)
{
    uint8_t i;

    C610_CAN = hcan;
    C610.home_active = 1;
    for (i = 0; i < C610_MOTOR_NUM; i++)
    {
        C610.motor[i].speed_rpm = 0;
        C610.motor[i].angle_deg = 0.0f;
        C610.motor[i].given_current = 0;
        C610.motor[i].temperature = 0;
        C610.motor[i].is_online = 0;
        C610.motor[i].mode = C610_MODE_HOME;
        C610.motor[i].target = 0.0f;
        C610.motor[i].last_encoder = 0;
        C610.motor[i].total_encoder = 0;
        C610.motor[i].last_feedback_tick = 0;
        C610.motor[i].first_feedback = 1;
        C610.motor[i].zero_encoder = STEER_ZERO_ENCODER;
        C610.motor[i].direction = 1;
        C610.motor[i].output_current = 0;

        PID_Init(&C610.motor[i].speed_pid, 10.0f, 0.02f, 2.00f, 5000.0f, -C610_MAX_CURRENT, C610_MAX_CURRENT, 0.0f, 10000.0f);
        PID_Init(&C610.motor[i].position_pid, 40.0f, 0.0f, 2.2f, 0.0f, -C610_MAX_POSITION_RPM, C610_MAX_POSITION_RPM, 0.0f, 0.0f);
    }
}

void C610_Receive(uint32_t can_id, uint8_t *data)
{
    uint8_t motor;
    uint16_t encoder;
    int32_t difference;

    if ((can_id < C610_FEEDBACK_ID_1) ||(can_id >= (C610_FEEDBACK_ID_1 + C610_MOTOR_NUM)))
    {
        return;
    }

    motor = (uint8_t)(can_id - C610_FEEDBACK_ID_1);
    encoder = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    C610.motor[motor].speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    C610.motor[motor].given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    C610.motor[motor].temperature = data[6];
    C610.motor[motor].last_feedback_tick = HAL_GetTick();
    C610.motor[motor].is_online = 1;

    if (C610.motor[motor].first_feedback != 0)
    {
        C610.motor[motor].last_encoder = encoder;
        difference = (int32_t)encoder - (int32_t)C610.motor[motor].zero_encoder;

        if (difference > C610_ENCODER_HALF) 
            difference -= C610_ENCODER_RANGE;
        if (difference < -C610_ENCODER_HALF) 
            difference += C610_ENCODER_RANGE;

        C610.motor[motor].total_encoder = difference * C610.motor[motor].direction;
        C610.motor[motor].angle_deg = (float)C610.motor[motor].total_encoder * 360.0f /
                                      ((float)C610_ENCODER_RANGE * C610_GEAR_RATIO);
        C610.motor[motor].first_feedback = 0;
        return;
    }

    difference = (int32_t)encoder - (int32_t)C610.motor[motor].last_encoder;
    if (difference > C610_ENCODER_HALF)
    {
        difference -= C610_ENCODER_RANGE;
    }
    else if (difference < -C610_ENCODER_HALF)
    {
        difference += C610_ENCODER_RANGE;
    }

    C610.motor[motor].total_encoder += difference * C610.motor[motor].direction;
    C610.motor[motor].last_encoder = encoder;
    C610.motor[motor].angle_deg = (float)C610.motor[motor].total_encoder * 360.0f /((float)C610_ENCODER_RANGE * C610_GEAR_RATIO);
}

void C610_Set_Command(uint8_t motor_id, uint8_t mode, int16_t target)
{
    uint8_t motor;

    motor = 0;

    // if (C610.motor[motor].mode != mode)
    // {
    //     PID_Reset(&C610.motor[motor].speed_pid);
    //     PID_Reset(&C610.motor[motor].position_pid);
    // }
    C610.motor[motor].mode = mode;

    if (mode == C610_MODE_CAPTURE_ZERO)
    {
        C610.motor[motor].zero_encoder = C610.motor[motor].last_encoder;
        C610.motor[motor].total_encoder = 0;
        C610.motor[motor].angle_deg = 0.0f;
        C610.motor[motor].mode = C610_MODE_STOP;
        C610.motor[motor].target = 0.0f;

        PID_Reset(&C610.motor[motor].speed_pid);
        PID_Reset(&C610.motor[motor].position_pid);

    }
    else if (mode == C610_MODE_HOME)
    {
        C610.motor[motor].target = STEER_OFFSET_DEG;
        //C610.motor[motor].target = C610_HOME_TARGET_DEG;
    }
    else
    {
        C610.motor[motor].target = (mode == C610_MODE_Position) ? ((float)target / 100.0f) : (float)target;
    }
}



void C610_Control(void)
{
    uint8_t i;
    float target_rpm;
    float output;
    uint32_t now = HAL_GetTick();
    uint8_t home_active = 0;

    for (i = 0; i < C610_MOTOR_NUM; i++)
    {
        if ((now - C610.motor[i].last_feedback_tick) > C610_FEEDBACK_TIMEOUT_MS)
        {
            C610.motor[i].is_online = 0;//不在线 输出0
        }

        output = 0.0f;
        if ((C610.motor[i].is_online) && (C610.motor[i].mode == C610_MODE_SPEED))
        {
            output = PID_Calculate(&C610.motor[i].speed_pid, C610.motor[i].target,(float)C610.motor[i].speed_rpm);
        }
        else if ((C610.motor[i].is_online) && (C610.motor[i].mode == C610_MODE_Position))
        {
            target_rpm = PID_Calculate(&C610.motor[i].position_pid, C610.motor[i].target, C610.motor[i].angle_deg);
            output = PID_Calculate(&C610.motor[i].speed_pid, target_rpm,(float)C610.motor[i].speed_rpm);
        }
        else if ((C610.motor[i].is_online) && (C610.motor[i].mode == C610_MODE_HOME))
        {
            home_active = 1;
            // if (((C610.motor[i].angle_deg - C610.motor[i].target) < C610_HOME_FINISH_DEG) &&
            //     ((C610.motor[i].angle_deg - C610.motor[i].target) > -C610_HOME_FINISH_DEG) &&
            //     (C610.motor[i].speed_rpm < C610_HOME_FINISH_RPM) &&
            //     (C610.motor[i].speed_rpm > -C610_HOME_FINISH_RPM))
            if (((C610.motor[i].angle_deg) < C610_HOME_FINISH_DEG) &&
                ((C610.motor[i].angle_deg) > -C610_HOME_FINISH_DEG) &&
                (C610.motor[i].speed_rpm < C610_HOME_FINISH_RPM) &&
                (C610.motor[i].speed_rpm > -C610_HOME_FINISH_RPM))
            {
                C610.motor[i].mode = C610_MODE_STOP;
                C610.motor[i].target = 0.0f;
                PID_Reset(&C610.motor[i].speed_pid);
                PID_Reset(&C610.motor[i].position_pid);
            }
            else
            {
                target_rpm = PID_Calculate(&C610.motor[i].position_pid, C610.motor[i].target, C610.motor[i].angle_deg);
                output = PID_Calculate(&C610.motor[i].speed_pid, target_rpm,(float)C610.motor[i].speed_rpm);
            }
        }
        else
        {
            PID_Reset(&C610.motor[i].speed_pid);
            PID_Reset(&C610.motor[i].position_pid);
        }

        C610.motor[i].output_current = (int16_t)output;
    }

    C610.home_active = home_active;
    C610_Send_Group(C610_CURRENT_ID_1_4, 0);
}

void C610_Start_Home_All(void)
{
    uint8_t i;

    for (i = 0; i < C610_MOTOR_NUM; i++)
    {
        C610.motor[i].mode = C610_MODE_HOME;
        C610.motor[i].target = STEER_OFFSET_DEG;
        PID_Reset(&C610.motor[i].speed_pid);
        PID_Reset(&C610.motor[i].position_pid);
    }
    C610.home_active = 1;
}

void C610_Stop_All(void)
{
    uint8_t i;

    for (i = 0; i < C610_MOTOR_NUM; i++)
    {
        C610.motor[i].mode = C610_MODE_STOP;
        C610.motor[i].target = 0.0f;
        C610.motor[i].output_current = 0;
        PID_Reset(&C610.motor[i].speed_pid);
        PID_Reset(&C610.motor[i].position_pid);
    }
    
    C610.home_active = 0;
    C610_Send_Group(C610_CURRENT_ID_1_4, 0);
}
