#include "DJI_2006.h"

static void int16_packet(DJI_Motor *motor)
{
    motor->target_speed = (int16_t)(motor->ftarget_speed * 100.0f);
    motor->target_position = (int16_t)(motor->ftarget_position * 100.0f);
}

void DJI2006_Init(DJI_Motor *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    motor->hfdcan = hfdcan;
    motor->can_id = 0x300; 
    motor->motor_id = motor_id;
    motor->ftarget_speed = 0.0f;
    motor->ftarget_position = 0.0f;
    motor->target_speed = 0;
    motor->target_position = 0;
    motor->relative_position = 0;
    motor->actual_position = 0;
    motor->actual_speed = 0;
    motor->dji_state = false;
    motor->dji_feed_id = 0;
}

void DJI2006_speed(DJI_Motor *motor,float speed)
{
    uint8_t data[8];
    motor->mode = DJI_SPEED;
    motor->ftarget_speed = speed;
    int16_packet(motor);
    data[0]= motor->motor_id;
    data[1]= motor->mode;
    data[2]= (uint8_t)((motor->target_speed) >> 8);
    data[3]= (uint8_t)((motor->target_speed));
    for(uint8_t i=4;i<8;i++)
    {
        data[i]=0;
    }
    CAN_Transmit(motor->hfdcan,0,motor->can_id,data,8);
}

void DJI2006_position(DJI_Motor *motor,float position)
{
    uint8_t data[8];
    motor->mode = DJI_POSITION;
    motor->ftarget_position = position;
    int16_packet(motor);
    data[0]= motor->motor_id;
    data[1]= motor->mode;
    data[2]= (uint8_t)((motor->target_position) >> 8);
    data[3]= (uint8_t)((motor->target_position));
    for(uint8_t i=4;i<8;i++)
    {
        data[i]=0;
    }
    CAN_Transmit(motor->hfdcan,0,motor->can_id,data,8);
}

void DJI2006_stop(DJI_Motor *motor)
{
    uint8_t data[8];
    motor->mode = DJI_STOP;
    data[0]= motor->motor_id;
    data[1]= motor->mode;
    for(uint8_t i=2;i<8;i++)
    {
        data[i]=0;
    }
    CAN_Transmit(motor->hfdcan,0,motor->can_id,data,8);
}

void DJI2006_to_zero(DJI_Motor *motor)
{
    uint8_t data[8];
    motor->mode = DJI_to_zero;
    data[0]= motor->motor_id;
    data[1]= motor->mode;
    for(uint8_t i=2;i<8;i++)
    {
        data[i]=0;
    }
    CAN_Transmit(motor->hfdcan,0,motor->can_id,data,8);
}

void DJI2006_Analysisdata(DJI_Motor *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    if(motor ==NULL || rx_header == NULL || data == NULL||rx_header->DataLength != FDCAN_DLC_BYTES_8)
    {
        return;
    }
    if(rx_header->IdType == FDCAN_STANDARD_ID)
	{
		if(rx_header->Identifier == (0x390U + motor->motor_id ))
		{
            motor->relative_position = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
            motor->actual_position = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
            motor->actual_speed = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
            motor->dji_state = data[6];
            motor->dji_feed_id = data[7];
        }
    }
}
    