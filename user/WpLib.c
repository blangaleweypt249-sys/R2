#include "WpLib.h"

uint8_t My_CrcSum(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc += data[i];
    }
    crc &= 0xFF;
    return crc;
}

void RadToAngle(float rad, float *angle)
{
    *angle = rad * 180.0f / PI;
}

void AngleToRad(float angle, float *rad)
{
    *rad = angle * PI / 180.0f;
}