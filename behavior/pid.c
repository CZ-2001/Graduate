#include "pid.h"
#define ABS(x) ((x)>0? (x):(-(x)))
//宏定义限幅函数
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
            input = max;       \
        else if (input < -max) \
            input = -max;      \
    }
		
void Handle_Over_Zero(float *set, float *ref, float T) //过零结算函数
{
    if (*set - *ref > (T / 2)) // 4096 ：半圈机械角度
    {
        *ref += T;
    }
    else if (*set - *ref < -(T / 2))
    {
        *ref = *ref - T;
    }
}


void PID_Init(PidTypeDef *pid, unsigned char mode, float kp, float ki, float kd, float max_out, float max_iout, float diff_max, float diff_min)
{
    if (pid == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->diff_max = diff_max;
    pid->diff_min = diff_min;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

float PID_Calc(PidTypeDef *pid, float ref, float set)
{
    unsigned char index;
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->error[0] = set - ref;

    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
					LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DIFF)
    {
        if (ABS(pid->error[0]) > pid->diff_max)
            index = 0;
        else if (ABS(pid->error[0]) < pid->diff_min)
            index = 1;
        else
            index = (pid->diff_max - ABS(pid->error[0])) / (pid->diff_max - pid->diff_min);
        pid->Iout += pid->Ki * pid->error[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Kp * pid->error[0] + index * pid->Iout + pid->Kd * (pid->error[0] - pid->error[1]);
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}
float PID_Calc_add_limit(PidTypeDef *pid, float ref, float set, float T)
{
     unsigned char index;
    if (pid == NULL)
    {
        return 0.0f;
    }
    Handle_Over_Zero(&set, &ref, T);
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DIFF)
    {
        if (ABS(pid->error[0]) > pid->diff_max)
            index = 0;
        else if (ABS(pid->error[0]) < pid->diff_min)
            index = 1;
        else
            index = (pid->diff_max - ABS(pid->error[0])) / (pid->diff_max - pid->diff_min);
        pid->Iout += pid->Ki * pid->error[0];
        pid->out = pid->Kp * pid->error[0] + index * pid->Iout + pid->Kd * (pid->error[0] - pid->error[1]);
        LimitMax(pid->out, pid->max_out);
    }

    return pid->out;
}
void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->set = 0.0f;
}



float PID_CALCAL( PID_DOUBLE*PID, float angle_target, float angle_feedback, float speed_feedback, float inner_limit)
{
    float inner_output, outer_output;
    inner_output = PID_Calc_add_limit(&PID->inner, angle_feedback, angle_target, inner_limit);
    outer_output = PID_Calc(&PID->outer, speed_feedback, inner_output);
    //	PID->output=outer_output;
    return outer_output;  //最后计算出来 速度 
}
