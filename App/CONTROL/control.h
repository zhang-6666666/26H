/* 速度 PID + 航向 PID */
#ifndef CONTROL_H
#define CONTROL_H

#include "pid.h"

typedef enum {
    CTRL_STOP,       /* 电机停止 */
    CTRL_SPEED,      /* 纯速度（无转向） */
    CTRL_LINE,       /* 巡线 */
} CtrlMode;

void control_init(void);
void control_set_speed(float target_a, float target_b);   /* 编码器计数/控制周期 */
void control_update();
void control_set_mode(CtrlMode mode);
CtrlMode control_get_mode(void);

/* 读取当前状态（供串口调试用） */
float control_get_speed_a(void);
float control_get_speed_b(void);
float control_get_pwm_a(void);
float control_get_pwm_b(void);

/* 读取/写入 PID 参数 */
void control_get_pid_speed_a(float *kp, float *ki, float *kd);
void control_get_pid_speed_b(float *kp, float *ki, float *kd);
void control_set_pid_speed_a(float kp, float ki, float kd);
void control_set_pid_speed_b(float kp, float ki, float kd);

#endif
