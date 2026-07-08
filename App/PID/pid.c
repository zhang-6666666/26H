/* 位置式 PID — 标准公式 + 积分限幅防饱和 */
#include "pid.h"
#include <string.h>

void pid_init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
    memset(p, 0, sizeof(*p));
    p->kp      = kp;
    p->ki      = ki;
    p->kd      = kd;
    p->out_min = out_min;
    p->out_max = out_max;
}

void pid_target(PID_t *p, float tgt)
{
    p->target = tgt;
}

void pid_reset(PID_t *p)
{
    p->integral = 0;
    p->prev_err = 0;
}

float pid_compute(PID_t *p, float measurement, float dt)
{
    float err  = p->target - measurement;
    float p_out = p->kp * err;

    /* I */
    p->integral += err * dt;
    if (p->integral >  p->out_max) p->integral =  p->out_max;
    if (p->integral < -p->out_max) p->integral = -p->out_max;
    float i_out = p->ki * p->integral;

    /* D */
    float d_out = 0;
    if (dt > 0.000001f) d_out = p->kd * (err - p->prev_err) / dt;
    p->prev_err = err;

    /* 限幅 */
    float out = p_out + i_out + d_out;
    if (out > p->out_max) return p->out_max;
    if (out < p->out_min) return p->out_min;
    return out;
}
