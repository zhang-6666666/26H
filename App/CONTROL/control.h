/* 速度 PID + 航向 PID */
#ifndef CONTROL_H
#define CONTROL_H

#include "pid.h"

typedef enum {
    CTRL_STOP,       /* 电机停止 */
    CTRL_SPEED,      /* 纯速度（无转向） */
    CTRL_LINE,       /* 巡线 */
} CtrlMode;

void control_init(CtrlMode mode, float speed_l, float speed_r);
void control_update();
CtrlMode control_get_mode(void);

#endif
