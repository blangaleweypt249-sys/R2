#include "PID.h"
#include <math.h>

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float max_integral, float min_output, float max_output,
              float dead_zone, float integral_separated_threshold)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_integral = max_integral;
    pid->min_output = min_output;
    pid->max_output = max_output;

    pid->dead_zone = dead_zone;
    pid->integral_separated_threshold = integral_separated_threshold;

    PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid)
{
    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float PID_Calculate(PID_Controller *pid, float target, float feedback)
{
    pid->target = target;
    pid->feedback = feedback;
    pid->error = target - feedback;

    //累计误差
    if (fabsf(pid->error) <= pid->integral_separated_threshold)
    {
        pid->integral += pid->error;
        pid->integral = PID_Clamp(pid->integral,-pid->max_integral,pid->max_integral);
    }

    //输出值
    pid->output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * (pid->error - pid->previous_error);

    //输出限制
    pid->output = PID_Clamp(pid->output,pid->min_output,pid->max_output);

    //死区判断
    if (fabsf(pid->output) < pid->dead_zone)
        pid->output = 0.0f;

    //误差更新
    pid->previous_error = pid->error;
    return pid->output;
}


//分段式
void Step_PID_Init(Step_PID_Controller *pid,
                        PID_Step small_step,//小误差
                        PID_Step middle_step,
                        PID_Step large_step,
                        float small_threshold,//小误差阈值 绝对值
                        float large_threshold,
                        float max_integral,
                        float min_output,
                        float max_output,
                        float dead_zone,
                        float integral_separated_threshold)
{
    pid->small_step = small_step;
    pid->middle_step = middle_step;
    pid->large_step = large_step;
    pid->small_threshold = fabsf(small_threshold);
    pid->large_threshold = fabsf(large_threshold);

    PID_Init(&pid->controller,small_step.Kp, small_step.Ki, small_step.Kd,max_integral, min_output, max_output,dead_zone, integral_separated_threshold);
}

void Sept_PID_Reset(Step_PID_Controller *pid)
{
    PID_Reset(&pid->controller);
}

float Sept_PID_Calculate(Step_PID_Controller *pid,float target, float feedback)
{
    float step_error;
    PID_Step step_pid;

    step_error = fabsf(target - feedback);

    if (step_error <= pid->small_threshold)
        step_pid = pid->small_step;
    else if (step_error <= pid->large_threshold)
        step_pid = pid->middle_step;
    else
        step_pid = pid->large_step;

    pid->controller.Kp = step_pid.Kp;
    pid->controller.Ki = step_pid.Ki;
    pid->controller.Kd = step_pid.Kd;

    return PID_Calculate(&pid->controller, target, feedback);
}
