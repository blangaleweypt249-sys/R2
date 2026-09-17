#include "DJI_2006.h"
#include <string.h>

packet packet_data;

static uint32_t command_id(const DJI_Motor *motor, uint8_t command)
{
    return 0x300U | ((uint32_t)command << 4) | motor->motor_id;
}

void DJI2006_Init(DJI_Motor *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    motor->hfdcan = hfdcan;
    motor->can_id = 0x300U;
    motor->motor_id = motor_id;
    motor->target_speed = 0.0f;
    motor->target_position = 0.0f;
}

void DJI2006_speed(DJI_Motor *motor, float speed)
{
    uint8_t data[4];
    motor->mode = DJI_SPEED;
    motor->target_speed = speed;
    packet_data.value = motor->target_speed;
    for (uint8_t i = 0; i < 4; i++)
    {
        data[i] = packet_data.data[i];
    }
    CAN_Transmit(motor->hfdcan, 0, command_id(motor, DJI_SPEED), data, 4);
}

void DJI2006_position(DJI_Motor *motor, float position)
{
    uint8_t data[4];
    motor->mode = DJI_POSITION;
    motor->target_position = position;
    packet_data.value = motor->target_position;
    for (uint8_t i = 0; i < 4; i++)
    {
        data[i] = packet_data.data[i];
    }
    CAN_Transmit(motor->hfdcan, 0, command_id(motor, DJI_POSITION), data, 4);
}

void DJI2006_stop(DJI_Motor *motor)
{
    uint8_t data[4] = {0};
    motor->mode = DJI_STOP;
    CAN_Transmit(motor->hfdcan, 0, command_id(motor, DJI_STOP), data, 4);
}

void DJI2006_to_zero(DJI_Motor *motor)
{
    uint8_t data[4] = {0};
    motor->mode = DJI_to_zero;
    CAN_Transmit(motor->hfdcan, 0, command_id(motor, DJI_to_zero), data, 4);
}

void DJI2006_sign_zero(DJI_Motor *motor)
{
    uint8_t data[4] = {0};
    motor->mode = DJI_sign_zero;
    CAN_Transmit(motor->hfdcan, 0, command_id(motor, DJI_sign_zero), data, 4);
}

void DJI2006_Analysisdata(DJI_Motor *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    if (motor == NULL || rx_header == NULL || data == NULL ||
        (rx_header->DataLength != FDCAN_DLC_BYTES_4 &&
         rx_header->DataLength != FDCAN_DLC_BYTES_8))
    {
        return;
    }
    if (rx_header->IdType == FDCAN_STANDARD_ID)
    {
        if (rx_header->Identifier == (0x390U + motor->motor_id))
        {
            if (rx_header->DataLength == FDCAN_DLC_BYTES_4)
                memcpy(&motor->actual_angle, data, sizeof(motor->actual_angle));
        }
    }
}
