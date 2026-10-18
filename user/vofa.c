#include "vofa.h"

vofa_DJIdata motor;

float_u8 vofa_Send[VOFA_CHANNEL_COUNT];
static uint8_t send_txbuffer[VOFA_FRAME_SIZE];

static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

void Vofa_Init(UART_HandleTypeDef *huart)
{
    motor.huart = huart;
}

void Vofa_Send(vofa_DJIdata *data)
{
    if (data == NULL || data->huart == NULL)
        return;
    vofa_Send[0].f = data->target_speed;
    vofa_Send[1].f = data->actual_speed;
    vofa_Send[2].f = data->target_position;
    vofa_Send[3].f = data->actual_position;
    vofa_Send[4].f = data->target_current;
    vofa_Send[5].f = data->actual_current;

    for (uint8_t i = 0; i < VOFA_CHANNEL_COUNT; i++)
    {
        for (uint8_t j = 0; j < 4; j++)
        {
            send_txbuffer[i * 4 + j] = vofa_Send[i].u8[j];
        }
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        send_txbuffer[VOFA_CHANNEL_COUNT * 4 + i] = vofa_tail[i];
    }

    HAL_UART_Transmit_DMA(data->huart,send_txbuffer,VOFA_FRAME_SIZE);
}