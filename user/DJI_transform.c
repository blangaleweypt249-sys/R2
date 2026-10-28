#include "DJI_transform.h"
#include <string.h>

float_u8 packet_data;

static uint32_t command_id(const DJI_HlemMotor *motor, uint8_t command)
{
    return 0x300U | ((uint32_t)command << 4) | motor->motor_id;
}

void Helm2006_Init(DJI_HlemMotor *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    motor->hfdcan = hfdcan;
    motor->can_id = 0x300U;
    motor->motor_id = motor_id;
    motor->target_speed = 0.0f;
    motor->target_position = 0.0f;
}

void Helm2006_speed(DJI_HlemMotor *motor, float speed)
{
    uint8_t data[4];
    motor->mode = Helm_SPEED;
    motor->target_speed = speed;
    packet_data.f = motor->target_speed;
    for (uint8_t i = 0; i < 4; i++)
    {
        data[i] = packet_data.u8[i];
    }
    CAN_Transmit(motor->hfdcan, 0, command_id(motor, Helm_SPEED), data, 4);
}

void Helm2006_position(DJI_HlemMotor *motor, float position)
{
    uint8_t data[4];
    motor->mode = Helm_POSITION;
    motor->target_position = position;
    packet_data.f = motor->target_position;
    for (uint8_t i = 0; i < 4; i++)
    {
        data[i] = packet_data.u8[i];
    }
    CAN_Transmit(motor->hfdcan, 0, command_id(motor, Helm_POSITION), data, 4);
}

void Helm2006_stop(DJI_HlemMotor *motor)
{
    uint8_t data[4] = {0};
    motor->mode = Helm_STOP;
    CAN_Transmit(motor->hfdcan, 0, command_id(motor, Helm_STOP), data, 4);
}

void Helm2006_to_zero(DJI_HlemMotor *motor)
{
    uint8_t data[4] = {0};
    motor->mode = Helm_to_zero;
    CAN_Transmit(motor->hfdcan, 0, command_id(motor, Helm_to_zero), data, 4);
}

void Helm2006_sign_zero(DJI_HlemMotor *motor)
{
    uint8_t data[4] = {0};
    motor->mode = Helm_sign_zero;
    CAN_Transmit(motor->hfdcan, 0, command_id(motor, Helm_sign_zero), data, 4);
}

void Helm2006_Analysisdata(DJI_HlemMotor *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
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
            {
                memcpy(&motor->actual_angle, data, sizeof(motor->actual_angle));
                if (motor->num[0] >= 1000)
                    motor->num[0] = 0;
                else
                    motor->num[0]++;
            }
        }
    }
}
