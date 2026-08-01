/* 钢球平衡控制 — 双环 PID（位置环 + 速度环）*/
#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>
#include "pid.h"

/* 调参区：电机上电回零后，0 脉冲对应杆水平。 */
#define BALANCE_KP_POS                    1.90f
#define BALANCE_KP_VEL                   20.0f
#define BALANCE_TARGET_VEL_LIMIT_MM_S    400.0f
#define BALANCE_LEVEL_PULSE              0L
#define BALANCE_MAX_PULSE_OFFSET         590L
#define BALANCE_MOTOR_RPM                 700U
#define BALANCE_MOTOR_SIGN                1L

typedef struct {
    PID_T   pid_pos;        /* 外环：位置 → 目标速度 */
    PID_T   pid_vel;        /* 内环：速度 → 绝对位置偏移脉冲 */
    float   target_pos;     /* 目标位置（中心=0） */
    float   current_pos;    /* 当前位置 (mm) */
    float   target_vel;     /* 目标速度 (mm/s) */
    float   current_vel;    /* 实际速度 (mm/s)，K230 传入 */
    int32_t pulse_offset;   /* 相对水平零点的脉冲偏移 */
    int32_t target_pulse;   /* 发给驱动器的绝对目标脉冲 */
} BalanceCtrl;

void Balance_Init(BalanceCtrl *b);
void Balance_SetTarget(BalanceCtrl *b, float target_pos_mm);
void Balance_Update(BalanceCtrl *b, float cur_pos_mm, float cur_vel_mm_s);
void Balance_VisionLost(BalanceCtrl *b);

#endif
