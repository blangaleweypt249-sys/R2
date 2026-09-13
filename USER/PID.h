#ifndef __PID_H__
#define __PID_H__

//位置式 PID 
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float target;
    float feedback;
    float error;
    float previous_error;
    float integral;

    //设为 0 表示一直积分
    float integral_separated_threshold;
    float max_integral;
    float max_output;
    float min_output;
    float dead_zone;

    float output;
} PID_Controller;

typedef struct//分段
{
    float Kp;
    float Ki;
    float Kd;
} PID_Step;

typedef struct
{
    PID_Controller controller;//普通位置PID控制
    PID_Step small_step;      //小误差
    PID_Step middle_step;     //中等误差
    PID_Step large_step;      //大误差
    float small_threshold;    //小误差阈值 绝对值
    float large_threshold;    //大误差阈值
} Step_PID_Controller;


//位置
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,float max_integral, float min_output, float max_output,float dead_zone, float integral_separated_threshold);
void PID_Reset(PID_Controller *pid);
float PID_Calculate(PID_Controller *pid, float target, float feedback);

//分段
void Sept_PID_Reset(Step_PID_Controller *pid);
float Sept_PID_Calculate(Step_PID_Controller *pid,float target, float feedback);

void Step_PID_Init(Step_PID_Controller *pid,
                        PID_Step small_step,
                        PID_Step middle_step,
                        PID_Step large_step,
                        float small_threshold,
                        float large_threshold,
                        float max_integral,
                        float min_output,
                        float max_output,
                        float dead_zone,
                        float integral_separated_threshold);

#endif
