#include "helm_chassis.h"
#include "mymain.h"

#define half_dy 0.238        // 几何中心到轮子中心的y轴距离
#define half_dx 0.252        // 几何中心到轮子中心的x轴距离
#define motor_diameter 0.075 // 轮子半径
#define PI 3.14159265358979323846f

PID pid_CarAnale;
Angle car_angle;

helm_state helm_states = helm_idle;
calibration helm_calibration_data = {0}; // 光电门
volatile uint8_t ready_target_initialized = 0U;

static float wrap_pi(float angle)
{
    while (angle > PI)
    {
        angle -= 2.0f * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0f * PI;
    }
    return angle;
}

void helm_calculate(Speed *body_speed, Helm_chassis *helm_chassises)
{
    if (body_speed->vx == 0.0f && body_speed->vy == 0.0f && fabsf(body_speed->vw) == 0.0f)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            helm_chassises->helm_speed[i] = 0.0f;
        }
        helm_chassises->helm_angle[0] = atan2f(+half_dy, -half_dx);
        helm_chassises->helm_angle[1] = atan2f(-half_dy, -half_dx);
        helm_chassises->helm_angle[2] = atan2f(-half_dy, +half_dx);
        helm_chassises->helm_angle[3] = atan2f(+half_dy, +half_dx);
    }
    else
    {
        helm_chassises->helm_angle[0] = atan2f(body_speed->vy + half_dx * body_speed->vw, body_speed->vx - half_dy * body_speed->vw);
        helm_chassises->helm_angle[1] = atan2f(body_speed->vy - half_dx * body_speed->vw, body_speed->vx - half_dy * body_speed->vw);
        helm_chassises->helm_angle[2] = atan2f(body_speed->vy - half_dx * body_speed->vw, body_speed->vx + half_dy * body_speed->vw);
        helm_chassises->helm_angle[3] = atan2f(body_speed->vy + half_dx * body_speed->vw, body_speed->vx + half_dy * body_speed->vw);

        helm_chassises->helm_speed[0] = -sqrtf(powf(body_speed->vx + half_dy * body_speed->vw, 2) + powf(body_speed->vy - half_dx * body_speed->vw, 2)) * 60.0f / ((2.0f * PI) * motor_diameter);
        helm_chassises->helm_speed[1] = sqrtf(powf(body_speed->vx - half_dy * body_speed->vw, 2) + powf(body_speed->vy - half_dx * body_speed->vw, 2)) * 60.0f / ((2.0f * PI) * motor_diameter);
        helm_chassises->helm_speed[2] = -sqrtf(powf(body_speed->vx - half_dy * body_speed->vw, 2) + powf(body_speed->vy + half_dx * body_speed->vw, 2)) * 60.0f / ((2.0f * PI) * motor_diameter);
        helm_chassises->helm_speed[3] = sqrtf(powf(body_speed->vx + half_dy * body_speed->vw, 2) + powf(body_speed->vy + half_dx * body_speed->vw, 2)) * 60.0f / ((2.0f * PI) * motor_diameter);
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        float desired_angle = wrap_pi(helm_chassises->helm_angle[i] - PI / 2.0f);
        float actual_angle = dji_motor[i].actual_angle * PI / 180.0f;   //转弧度

        helm_chassises->delta[i] = wrap_pi(desired_angle - actual_angle);
        if (helm_chassises->delta[i] > PI / 2.0f)
        {
            helm_chassises->delta[i] -= PI;
            helm_chassises->helm_speed[i] *= -1.0f;
        }
        else if (helm_chassises->delta[i] < -PI / 2.0f)
        {
            helm_chassises->delta[i] += PI;
            helm_chassises->helm_speed[i] *= -1.0f;
        }

        helm_chassises->actual_helm_angle[i] = actual_angle;    //小板反馈的真实舵角
        helm_chassises->helm_angle[i] = actual_angle + helm_chassises->delta[i];
        helm_chassises->set_angle[i] = (helm_chassises->helm_angle[i] / PI) * 180.0f;  //转角度
    }
}

void DJI_calibration(void)
{

    while (helm_calibration_data.all_helm_success_flag == false)
    {
        /* ---------- 0号 ---------- */

        if (helm_calibration_data.helm_calibration_flag[0] == false)
        {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
            {
                dji_motor[0].calibration_valid = true;
                DJI2006_speed(&dji_motor[0], 0.0f);
                DJI2006_sign_zero(&dji_motor[0]);
                helm_calibration_data.helm_calibration_flag[0] = true;
            }
            else
            {
                DJI2006_speed(&dji_motor[0], 15.0f);
            }
        }
        else
        {
            DJI2006_speed(&dji_motor[0], 0.0f);
        }

        /* ---------- 1号 ---------- */

        if (helm_calibration_data.helm_calibration_flag[1] == false)
        {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
            {
                dji_motor[1].calibration_valid = true;
                DJI2006_speed(&dji_motor[1], 0.0f);
                DJI2006_sign_zero(&dji_motor[1]);
                helm_calibration_data.helm_calibration_flag[1] = true;
            }
            else
            {
                DJI2006_speed(&dji_motor[1], 15.0f);
            }
        }
        else
        {
            DJI2006_speed(&dji_motor[1], 0.0f);
        }

        /* ---------- 2号 ---------- */

        if (helm_calibration_data.helm_calibration_flag[2] == false)
        {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)
            {
                dji_motor[2].calibration_valid = true;
                DJI2006_speed(&dji_motor[2], 0.0f);
                DJI2006_sign_zero(&dji_motor[2]);
                helm_calibration_data.helm_calibration_flag[2] = true;
            }
            else
            {
                DJI2006_speed(&dji_motor[2], 15.0f);
            }
        }
        else
        {
            DJI2006_speed(&dji_motor[2], 0.0f);
        }

        /* ---------- 3号 ---------- */

        if (helm_calibration_data.helm_calibration_flag[3] == false)
        {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
            {
                dji_motor[3].calibration_valid = true;
                DJI2006_speed(&dji_motor[3], 0.0f);
                DJI2006_sign_zero(&dji_motor[3]);
                helm_calibration_data.helm_calibration_flag[3] = true;
            }
            else
            {
                DJI2006_speed(&dji_motor[3], 5.0f);
            }
        }
        else
        {
            DJI2006_speed(&dji_motor[3], 0.0f);
        }

        /* ---------- 检查是否全部完成 ---------- */

        if (helm_calibration_data.helm_calibration_flag[0] &&
            helm_calibration_data.helm_calibration_flag[1] &&
            helm_calibration_data.helm_calibration_flag[2] &&
            helm_calibration_data.helm_calibration_flag[3])
        {
            helm_calibration_data.all_helm_success_flag = true;

            helm_states = helm_calibration_success;
        }

        osDelay(10);
    }
}

void helm_chassis_ready(void)
{
    if (!ready_target_initialized)
    {
        if (!dji_motor[0].calibration_valid || !dji_motor[1].calibration_valid ||
            !dji_motor[2].calibration_valid || !dji_motor[3].calibration_valid)
            return;
        for (uint16_t count = 0; count < 200; count++)
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                DJI2006_to_zero(&dji_motor[i]);    //必须保证tozero的signzero成功，这期间不能切换模式
            }

            osDelay(10);
        }
        ready_target_initialized = 1U;
    }
}

void World_to_body(Speed *world_speed, Speed *body_speed, float imu_angle)
{
    float cos_angle = cosf(imu_angle);
    float sin_angle = sinf(imu_angle);

    body_speed->vx = world_speed->vx * cos_angle + world_speed->vy * sin_angle;
    body_speed->vy = -world_speed->vx * sin_angle + world_speed->vy * cos_angle;
    body_speed->vw = world_speed->vw;
}

void car_angle_maintain(Speed *world_speed, Angle *angle)
{
    angle->angle_out = calc_pid(&pid_CarAnale, angle->actual_angle, angle->target_angle);
    world_speed->actual_vw = angle->angle_out;
}
