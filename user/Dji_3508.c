#include "Dji_Master.h"

#define DJI3508_EXTERNAL_RATIO 1.0f
#define DJI3508_CURRENT_LIMIT 16384.0f
#define DJI3508_SPEED_LIMIT 480.0f

static int16_t current[8];

static int16_t limit_current(float current)
{
    if (current > 16384.0f)
        current = 16384.0f;
    else if (current < -16384.0f)
        current = -16384.0f;
    return (int16_t)current;
}

void DJI_3508_Init(DJI_Motor *Motor, FDCAN_HandleTypeDef *hcan, uint8_t motor_id)
{
    Motor->hcan = hcan;
    Motor->motor_id = motor_id;
    Motor->feedback_id = 0x200 + motor_id;
    Motor->target_speed = 0.0f;
    Motor->target_position = 0.0f;
    Motor->target_current = 0;
    Motor->first_feedback = 0;

    pid_init(&Motor->speed_pid,
             0.0f,
             10.0f, 0.0f, 0.0f,
             10000.0f, 3000.0f, 0.0f);

    pid_init(&Motor->position_pid,
             0.0f,
             5.0f, 0.0f, 0.0f,
             3000.0f, 0.0f, 0.5f);
}

void DJI_3508CAN_Trans_1(FDCAN_HandleTypeDef *hcan)
{
    uint8_t current_data[8];
    current_data[0] = current[0] >> 8;
    current_data[1] = current[0] & 0xFF;
    current_data[2] = current[1] >> 8;
    current_data[3] = current[1] & 0xFF;
    current_data[4] = current[2] >> 8;
    current_data[5] = current[2] & 0xFF;
    current_data[6] = current[3] >> 8;
    current_data[7] = current[3] & 0xFF;
    CAN_Transmit(hcan, 0, 0x200, current_data, 8);
}

void DJI_3508CAN_Trans_2(FDCAN_HandleTypeDef *hcan)
{
    uint8_t current_data[8];
    current_data[0] = current[4] >> 8;
    current_data[1] = current[4] & 0xFF;
    current_data[2] = current[5] >> 8;
    current_data[3] = current[5] & 0xFF;
    current_data[4] = current[6] >> 8;
    current_data[5] = current[6] & 0xFF;
    current_data[6] = current[7] >> 8;
    current_data[7] = current[7] & 0xFF;
    CAN_Transmit(hcan, 0, 0x1FF, current_data, 8);
}

void DJI_3508SetCurrent(DJI_Motor *Motor, int16_t target_current)
{
    if (Motor == NULL || Motor->motor_id < 1 ||
        Motor->motor_id > 8)
        return;

    current[Motor->motor_id - 1] = limit_current((float)target_current);
}

void DJI_3508SetSpeed(DJI_Motor *Motor, float speed_rpm)
{
    float current;

    if (Motor == NULL)
        return;

    if (speed_rpm > DJI3508_SPEED_LIMIT)
        speed_rpm = DJI3508_SPEED_LIMIT;
    else if (speed_rpm < -DJI3508_SPEED_LIMIT)
        speed_rpm = -DJI3508_SPEED_LIMIT;
    Motor->target_speed = speed_rpm;
    current = calc_pid(&Motor->speed_pid,
                       (float)Motor->speed_rpm,
                       Motor->target_speed);

    DJI_3508SetCurrent(Motor, (int16_t)current);
}

void DJI_3508SetPosition(DJI_Motor *Motor, float position_deg)
{
    float target_speed;
    float target_current;

    if (Motor == NULL)
        return;

    Motor->target_position = position_deg;

    /* 位置外环：输出目标转速 */
    target_speed = calc_pid(&Motor->position_pid,
                            Motor->angle_deg,
                            Motor->target_position);

    /* 速度内环：输出目标电流 */
    target_current = calc_pid(&Motor->speed_pid,
                              (float)Motor->speed_rpm,
                              target_speed);

    DJI_3508SetCurrent(Motor, (int16_t)target_current);
}

void DJI_3508ReadDate(DJI_Motor *Motor, uint8_t rx_data[8], FDCAN_RxHeaderTypeDef *rx_header)
{
    int32_t delta = 0;
    if (rx_header->Identifier != Motor->feedback_id)
        return;
    if ((rx_header->IdType != FDCAN_STANDARD_ID) ||
        (rx_header->DataLength != FDCAN_DLC_BYTES_8))
        return;
    Motor->encoder = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    Motor->speed_rpm = (float)(int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
    Motor->torque_current = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
    if (Motor->first_feedback == 0)
    {
        Motor->first_feedback = 1;
        Motor->last_encoder = Motor->encoder;
    }
    else
    {
        delta = (int32_t)Motor->encoder - Motor->last_encoder;
        if (delta > 4096)
            delta -= 8192;
        else if (delta < -4096)
            delta += 8192;
        Motor->total_encoder += delta;
        Motor->last_encoder = Motor->encoder;
    }
    Motor->angle_deg = (float)Motor->total_encoder * 360.0f /
                       (8192.0f * DJI3508_EXTERNAL_RATIO);

    if (Motor->num[0] >= 1000)
        Motor->num[0] = 0;
    else
        Motor->num[0]++;
}
