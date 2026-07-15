/* TB6612 双路电机驱动 — 配置结构体 + 句柄模式 */
#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    GPIO_TypeDef *ain1_port, *ain2_port;
    uint16_t      ain1_pin,  ain2_pin;
    TIM_HandleTypeDef *htim;
    uint32_t      pwm_channel;
    uint16_t      pwm_period;
} Motor_Config;

typedef struct {
    Motor_Config cfg;
} Motor;

void Motor_Init(Motor *m, const Motor_Config *cfg);
void Motor_Run(Motor *m, int16_t permil);
void Motor_Coast(Motor *m);
void Motor_Brake(Motor *m);

extern Motor motor_left;
extern Motor motor_right;

#endif
