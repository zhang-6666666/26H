/* 速度 PID + 航向 PID */
#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "jy901p.h"
#include "motor.h"
#include <line.h>


static PID_T pid_speed_a, pid_speed_b, pid_yaw;
static float base_speed_a, base_speed_b;
static float pwm_a_out, pwm_b_out;  /* 供串口调试读取 */

void control_init(void)
{
    pid_init(&pid_speed_a, 10.0f, 0.5f, 0.0f, 0.0f, 1000.0f);
    pid_init(&pid_speed_b, 10.0f, 0.5f, 0.0f, 0.0f, 1000.0f);
    pid_init(&pid_yaw,    1.0f, 0.0f, 0.0f, 0.0f, 1000.0f);
}

void control_set_speed(float target_a, float target_b)
{
    base_speed_a = target_a;
    base_speed_b = target_b;
}

void control_set_yaw(float target)
{
    while (target >  180.0f) target -= 360.0f;
    while (target < -180.0f) target += 360.0f;
    pid_set_target(&pid_yaw, target);
}

void control_update(float dt)
{
    Encoder_Update(&encoder_left);
    Encoder_Update(&encoder_right);

    float steer = (float)pid_calculate_angle_positional(&pid_yaw, angle_y);

    pid_set_target(&pid_speed_a, base_speed_a - steer);
    pid_set_target(&pid_speed_b, base_speed_b + steer);

    pwm_a_out = pid_calculate_positional(&pid_speed_a, encoder_left.speed_cm_s);
    pwm_b_out = pid_calculate_positional(&pid_speed_b, encoder_right.speed_cm_s);

    Motor_Run(&motor_left,  (int16_t)pwm_a_out);
    Motor_Run(&motor_right, (int16_t)pwm_b_out);
}

/* ===================== 状态读取 ===================== */

float control_get_speed_a(void) { return base_speed_a; }
float control_get_speed_b(void) { return base_speed_b; }
float control_get_yaw(void)     { return pid_yaw.target; }
float control_get_pwm_a(void)   { return pwm_a_out; }
float control_get_pwm_b(void)   { return pwm_b_out; }

void control_get_pid_speed_a(float *kp, float *ki, float *kd) { pid_get_params(&pid_speed_a, kp, ki, kd); }
void control_get_pid_speed_b(float *kp, float *ki, float *kd) { pid_get_params(&pid_speed_b, kp, ki, kd); }
void control_get_pid_yaw(float *kp, float *ki, float *kd)     { pid_get_params(&pid_yaw, kp, ki, kd); }

void control_set_pid_speed_a(float kp, float ki, float kd) { pid_set_params(&pid_speed_a, kp, ki, kd); }
void control_set_pid_speed_b(float kp, float ki, float kd) { pid_set_params(&pid_speed_b, kp, ki, kd); }
void control_set_pid_yaw(float kp, float ki, float kd)     { pid_set_params(&pid_yaw, kp, ki, kd); }
