/* 速度 PID + 航向 PID */
#ifndef CONTROL_H
#define CONTROL_H

void control_init(void);
void control_set_speed(float target_a, float target_b);   /* 编码器计数/控制周期 */
void control_set_yaw(float target);     /* 目标航向角度 (°) */
void control_update(float dt);          /* dt=控制周期(秒) */

#endif
