/* 速度 PID + 航向 PID */
#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "jy901p.h"
#include "motor.h"

static PID_T pid_speed_a, pid_speed_b, pid_yaw;
static float base_speed_a, base_speed_b;

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
    /* 更新编码器（读 CNT → 方向修正 → 清零） */
    Encoder_Update(&left_encoder);
    Encoder_Update(&right_encoder);

    float steer = (float)pid_calculate_angle_positional(&pid_yaw, angle_y);
    // steer = steer / 4.0f; // 将 PID 输出缩小到合理范围

    pid_set_target(&pid_speed_a, base_speed_a - steer);
    pid_set_target(&pid_speed_b, base_speed_b + steer);

    // pid_set_target(&pid_speed_a, base_speed_a);
    // pid_set_target(&pid_speed_b, base_speed_b);
    float motor_a_pwm_in = pid_calculate_positional(&pid_speed_a, (float)left_encoder.speed_cm_s);
    float motor_b_pwm_in = pid_calculate_positional(&pid_speed_b, (float)right_encoder.speed_cm_s);

    motor_a_run((int16_t)motor_a_pwm_in);
    motor_b_run((int16_t)motor_b_pwm_in);

    // motor_a_run((int16_t)200-steer);
    // motor_b_run((int16_t)200+steer);

}
