#ifndef __DT35_H__
#define __DT35_H__

#include "mymain.h"

typedef union 
{
    uint8_t  u8[16];
    float    flo[4];
}U8_to_Flo;

typedef struct 
{
	float actual_dt35_distance;
	float target_dt35_distance;
}DT35_Move;

void DT35_Analysis(uint8_t *data);

#endif
