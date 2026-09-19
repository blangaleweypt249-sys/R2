#ifndef __HELM_CHASSIS_H__
#define __HELM_CHASSIS_H__

#include <stdbool.h>
#include <stdint.h>
#include "pid.h"
#include "DJI_2006.h"

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
    float actual_helm_angle[4]; // 上一次完整目标角，单位为弧度
    float delta[4];
    float set_angle[4];
}Helm_chassis;

typedef enum
{
    helm_idle = 0,
    helm_calibration,
    helm_calibration_success
}helm_state;

typedef struct
{
	float actual_angle;
	float target_angle;
	float angle_out;
	float last_angle;
}Angle;

typedef struct
{
  bool all_helm_success_flag;
  bool helm_calibration_flag[4];
}calibration;

extern volatile uint8_t ready_target_initialized;
extern PID pid_CarAnale;
extern Angle car_angle;

void DJI_calibration(void);
void helm_calculate(Speed *body_speed,Helm_chassis *helm_chassises);
void helm_chassis_ready(void);
void car_angle_maintain(Speed *world_speed, Angle *angle);
void World_to_body(Speed *world_speed, Speed *body_speed, float imu_angle);

#endif
