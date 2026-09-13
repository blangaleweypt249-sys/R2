#include "AS5047P.h"

#define AS5047P_NOP        0x0000//无操作
#define AS5047P_ZPOSM      0x0016//零位角度高位
#define AS5047P_ZPOSL      0x0017//零位角度低位
#define AS5047P_ANGLECOM   0x3FFF//测量角度 动态误差补偿
#define AS5047P_DATA_MASK  0x3FFF//低14位是有效数据
#define AS5047P_READ_CMD   0x4000//读命令标志
#define AS5047P_ERROR_FLAG 0x4000//接收数据中的错误标志

//统计16位数据中1的个数 偶数返回1 奇数返回0
static uint8_t AS5047P_EvenParity(uint16_t data)
{
    uint8_t count = 0;

    while (data != 0)
    {
        if ((data & 1) != 0)
        {
            count++;
        }
        data >>= 1;
    }

    return (count % 2) == 0;
}

//发送命令前自动补偶校验位
static uint16_t AS5047P_AddParity(uint16_t data)
{
    data &= 0x7FFF;
    if (AS5047P_EvenParity(data) == 0)
    {
        data |= 0x8000;
    }
    return data;
}

static uint16_t AS5047P_Transfer(AS5047P_HandleTypeDef *user,uint16_t send_data)
{
    uint16_t receive_data = 0;

    HAL_GPIO_WritePin(user->CS_GPIOx, user->CS_GPIO_Pin,GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive(user->hspi,(uint8_t *)&send_data,(uint8_t *)&receive_data,1, 10) != HAL_OK)
    {
        user->error = 1;//通信失败
    }

    HAL_GPIO_WritePin(user->CS_GPIOx, user->CS_GPIO_Pin, GPIO_PIN_SET);
    return receive_data;
}

//AS5047P读数据 两帧 先地址 再数据 
static uint16_t AS5047P_ReadRegister(AS5047P_HandleTypeDef *user,uint16_t address)
{
    uint16_t command;
    uint16_t data;

    user->error = 0;
    command = AS5047P_AddParity(AS5047P_READ_CMD | (address & AS5047P_DATA_MASK));
    AS5047P_Transfer(user, command);
    data = AS5047P_Transfer(user, AS5047P_AddParity(AS5047P_READ_CMD | AS5047P_NOP));

    if (user->error != 0)
    {
        return 0;
    }
    if (AS5047P_EvenParity(data) == 0)
    {
        user->error = 2;//收到的数据不完整
        return 0;
    }
    if ((data & AS5047P_ERROR_FLAG) != 0)
    {
        user->error = 3;//传感器报错
        return 0;
    }
    return data & AS5047P_DATA_MASK;
}

static void AS5047P_WriteRegister(AS5047P_HandleTypeDef *user,uint16_t address, uint16_t data)
{
    uint16_t command;

    user->error = 0;
    command = AS5047P_AddParity(address & AS5047P_DATA_MASK);
    AS5047P_Transfer(user, command);
    AS5047P_Transfer(user, AS5047P_AddParity(data & AS5047P_DATA_MASK));
}

void AS5047P_Init(AS5047P_HandleTypeDef *user,SPI_HandleTypeDef *hspi,GPIO_TypeDef *CS_GPIOx,uint16_t CS_GPIO_Pin)
{
    user->hspi = hspi;
    user->CS_GPIOx = CS_GPIOx;
    user->CS_GPIO_Pin = CS_GPIO_Pin;
    user->raw_angle = 0;
    user->angle = 0;
    user->error = 0;

    HAL_GPIO_WritePin(CS_GPIOx, CS_GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    AS5047P_Read(user);
}

void AS5047P_Read(AS5047P_HandleTypeDef *user)
{
    user->raw_angle = AS5047P_ReadRegister(user, AS5047P_ANGLECOM);
    if (user->error == 0)
    {
        user->angle = user->raw_angle * 360.0f / 16384.0f;
    }
}

void AS5047P_SetZero(AS5047P_HandleTypeDef *user, uint16_t zero)
{
    uint16_t zero_pos;

    zero &= AS5047P_DATA_MASK;
    zero_pos = AS5047P_ReadRegister(user, AS5047P_ZPOSL) & 0x00C0;
    AS5047P_WriteRegister(user, AS5047P_ZPOSM, zero >> 6);
    AS5047P_WriteRegister(user, AS5047P_ZPOSL, zero_pos | (zero & 0x003F));
}
