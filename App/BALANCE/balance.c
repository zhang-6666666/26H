/* 钢球平衡控制：位置 P -> 目标球速，速度 P -> 绝对电机位置。 */
#include "balance.h"
#include "motor_bujin.h"

/*
 *  架构：
 *  外环（位置）：pos_error → Kp_pos → target_vel（球该滚多快回到中心）
 *  内环（速度）：vel_error → Kp_vel → 相对水平零点的脉冲偏移
 *  电机驱动器：绝对位置闭环，0 脉冲为上电回零后的水平位置
 */

static int32_t round_to_i32(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f)
                           : (int32_t)(value - 0.5f);
}

void Balance_Init(BalanceCtrl *b)
{
    pid_init(&b->pid_pos, BALANCE_KP_POS, 0.0f, 0.0f, 0.0f,
             BALANCE_TARGET_VEL_LIMIT_MM_S);

    pid_init(&b->pid_vel, BALANCE_KP_VEL, 0.0f, 0.0f, 0.0f,
             (float)BALANCE_MAX_PULSE_OFFSET);

    b->target_pos = 0.0f;
    b->current_pos = 0.0f;
    b->target_vel = 0.0f;
    b->current_vel = 0.0f;
    b->pulse_offset = 0;
    b->target_pulse = BALANCE_LEVEL_PULSE;
}

void Balance_Update(BalanceCtrl *b, float cur_pos_mm, float cur_vel_mm_s)
{
    float raw_pulse_offset;

    b->current_pos = cur_pos_mm;
    b->current_vel = cur_vel_mm_s;

    pid_set_target(&b->pid_pos, b->target_pos);
    b->target_vel = pid_calculate_positional(&b->pid_pos, b->current_pos);

    pid_set_target(&b->pid_vel, b->target_vel);
    raw_pulse_offset = pid_calculate_positional(&b->pid_vel,
                                                 b->current_vel);

    b->pulse_offset = BALANCE_MOTOR_SIGN * round_to_i32(raw_pulse_offset);
    b->target_pulse = BALANCE_LEVEL_PULSE + b->pulse_offset;
    Motor_Step_MoveTo(b->target_pulse, BALANCE_MOTOR_RPM);
}

void Balance_VisionLost(BalanceCtrl *b)
{
    /* Do not integrate stale vision data after an invalid frame or timeout. */
    pid_reset(&b->pid_pos);
    pid_reset(&b->pid_vel);
    b->target_vel = 0.0f;
    b->current_vel = 0.0f;
    b->pulse_offset = 0;
    b->target_pulse = BALANCE_LEVEL_PULSE;
    Motor_Step_Stop();
}
