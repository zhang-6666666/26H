/* 钢球平衡控制 — 步进电机 PID 闭环 */
#include "balance.h"
#include "motor_bujin.h"

void Balance_Init(BalanceCtrl *b)
{
    /* Kp/Ki/Kd 待实际调参，先用保守值 */
    pid_init(&b->pid, 1.0f, 0.0f, 0.0f, 0.0f, 3000.0f);
    b->target  = 0.0f;
    b->current = 0.0f;
    b->output  = 0.0f;
}

void Balance_SetTarget(BalanceCtrl *b, float pos)
{
    b->target = pos;
}

void Balance_Update(BalanceCtrl *b, float current_pos)
{
    b->current = current_pos;
    pid_set_target(&b->pid, b->target);
    b->output = pid_calculate_positional(&b->pid, b->current);
    // Motor_Step_MoveRel((int32_t)b->output,30);
}
