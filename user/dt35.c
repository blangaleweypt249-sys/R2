#include "dt35.h"

U8_to_Flo distance;
DT35_Move dt35_front;
DT35_Move dt35_back;
DT35_Move dt35_left;
DT35_Move dt35_right;

void DT35_Analysis(uint8_t *data)
{
    uint16_t crc = 0;
    if (data[0] == 0x35 && data[18] == 0x53)
    {
        for (uint8_t i = 1; i <= 16; i++)
        {
            crc += data[i];
        }
        if ((crc & 0xFF) == data[17])
        {
            for(uint8_t i =1;i<=16;i++)
            {
                distance.u8[i-1] = data[i];
            }
            dt35_front.actual_dt35_distance = distance.flo[3];
            dt35_back.actual_dt35_distance = distance.flo[0];
            dt35_left.actual_dt35_distance = distance.flo[1];
            dt35_right.actual_dt35_distance = distance.flo[2];
        }
    }
}
