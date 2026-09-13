#ifndef __AS5047P_H__
#define __AS5047P_H__

#include "main.h"
#include "spi.h"
#include "gpio.h"

typedef struct
{
    SPI_HandleTypeDef *hspi;//SPI句柄
    GPIO_TypeDef *CS_GPIOx;
    uint16_t CS_GPIO_Pin;//自定义引脚

    uint16_t raw_angle; //0~16383  
    float angle;        //0~360 度 (raw_angle/16384)*360
    uint8_t error;      //0正常 1通信失败 2奇偶校验错误 3传感器报错

} AS5047P_HandleTypeDef;

void AS5047P_Init(AS5047P_HandleTypeDef *user,SPI_HandleTypeDef *hspi,GPIO_TypeDef *CS_GPIOx,uint16_t CS_GPIO_Pin);

//读角度 deg
void AS5047P_Read(AS5047P_HandleTypeDef *user);

//调试时用 临时改寄存器 重新上电恢复
void AS5047P_SetZero(AS5047P_HandleTypeDef *user, uint16_t zero);

#endif
