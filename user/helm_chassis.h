#ifndef __HELM_CHASSIS_H__
#define __HELM_CHASSIS_H__
#include "mymain.h"

typedef struct
{
	float vx;
	float vy;
	float vw;
	float actual_vx;
	float actual_vy;
	float actual_vw;
}Speed;

typedef struct
{
    float helm_angle[4];
    float helm_speed[4];
}Helm_chassis;

typedef enum
{
    helm_idle = 0,
    helm_calibration,
    helm_calibration_success
}helm_state;

typedef struct
{
  bool all_helm_success_flag;
  bool helm_calibration_flag[4];
}calibration;

void DJI_calibration(void);
void helm_calculate(Speed *body_speed,Helm_chassis *helm_chassises);
void helm_chassis_ready(void);

#endif