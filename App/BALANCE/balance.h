/* 钢球平衡控制 — 步进电机 PID 闭环
   外环：球位置 → PID → 电机转速/步数
   步进电机内部已有位置闭环，外层只需一个 PID
*/
#ifndef BALANCE_H
#define BALANCE_H

#include "pid.h"

typedef struct {
    PID_T   pid;            /* 位置环 PID */
    float   target;         /* 目标位置（0=平台中心） */
    float   current;        /* 当前位置（K230 传入） */
    float   output;         /* PID 输出（转速 RPM 或步数） */
} BalanceCtrl;

void Balance_Init(BalanceCtrl *b);
void Balance_SetTarget(BalanceCtrl *b, float pos);
void Balance_Update(BalanceCtrl *b, float current_pos);   /* 50ms 调用 */

#endif
