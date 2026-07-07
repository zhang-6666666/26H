/* TB6612 双路电机驱动 — 低耦合，只依赖 HAL 库 */
#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 初始化：保存 TIM 句柄并启动 PWM，GPIO 由 CubeMX 已配置 */
void motor_init(TIM_HandleTypeDef *htim);

/* 电机 A 运行：permil = -1000 ~ +1000（负值反转，0 滑行停止） */
void motor_a_run(int16_t permil);

/* 电机 B 运行：同上 */
void motor_b_run(int16_t permil);

/* 电机 A/B 滑行停止（IN1=IN2=0，电机自由转动） */
void motor_a_coast(void);
void motor_b_coast(void);

/* 电机 A/B 刹车（IN1=IN2=1，电机短接制动） */
void motor_a_brake(void);
void motor_b_brake(void);

#endif
