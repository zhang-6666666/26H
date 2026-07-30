/* 速度 PID + 模式切换 */
#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "line_follow.h"
#include "gray.h"

#define STOP_CNT  3    /* ≥3 路黑线 → 停车 */

static PID_T pid_speed_a, pid_speed_b;
static float base_speed_a, base_speed_b;
static float pwm_a_out, pwm_b_out;
static CtrlMode s_mode = CTRL_LINE;

void control_init(CtrlMode mode, float speed_l, float speed_r)
{
    /* 清零所有状态，每次启动任务都是冷启动 */
    Encoder_Reset(&encoder_left);
    Encoder_Reset(&encoder_right);
    LineFollow_Reset();
    pid_init(&pid_speed_a, 12.0f, 0.3f, 0.0f, 0.0f, 1000.0f);
    pid_init(&pid_speed_b, 12.0f, 0.3f, 0.0f, 0.0f, 1000.0f);

    s_mode       = mode;
    base_speed_a = speed_l;
    base_speed_b = speed_r;
}

void control_update(void)
{
    CtrlMode mode = s_mode;

    if (mode == CTRL_STOP) {
        Motor_Coast(&motor_left);
        Motor_Coast(&motor_right);
        return;
    }

    Encoder_Update(&encoder_left);
    Encoder_Update(&encoder_right);

    float steer = 0.0f;

    switch (mode) {
    case CTRL_SPEED:
        steer = 0.0f;
        break;

    case CTRL_LINE:
        {
            steer = LineFollow_Update();

            /* 停车线检测 */
            uint8_t d = gs.digital;
            uint8_t black_cnt = 0;
            for (uint8_t i = 0; i < 8; i++) {
                if (d & (1 << i)) black_cnt++;
            }
            if (black_cnt >= STOP_CNT) {
                s_mode = CTRL_STOP;
                Motor_Coast(&motor_left);
                Motor_Coast(&motor_right);
                return;
            }
        }
        break;

    default:
        break;
    }

    pid_set_target(&pid_speed_a, base_speed_a + steer);
    pid_set_target(&pid_speed_b, base_speed_b - steer);

    pwm_a_out = pid_calculate_positional(&pid_speed_a, encoder_left.speed_cm_s);
    pwm_b_out = pid_calculate_positional(&pid_speed_b, encoder_right.speed_cm_s);

    Motor_Run(&motor_left,  (int16_t)pwm_a_out);
    Motor_Run(&motor_right, (int16_t)pwm_b_out);
}

CtrlMode control_get_mode(void) { return s_mode; }

/* ===================== 状态读取 ===================== */


void control_get_pid_speed_a(float *kp, float *ki, float *kd) { pid_get_params(&pid_speed_a, kp, ki, kd); }
void control_get_pid_speed_b(float *kp, float *ki, float *kd) { pid_get_params(&pid_speed_b, kp, ki, kd); }

void control_set_pid_speed_a(float kp, float ki, float kd) { pid_set_params(&pid_speed_a, kp, ki, kd); }
void control_set_pid_speed_b(float kp, float ki, float kd) { pid_set_params(&pid_speed_b, kp, ki, kd); }
