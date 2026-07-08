/* 编码器读取 — 硬件 TIM 编码器模式，零中断，直接读 CNT */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* 初始化：启动 TIM2/TIM4 编码器模式计数 */
void encoder_init(void);

/* 读取编码器原始计数值（4 倍频） */
int32_t encoder_a_get(void);
int32_t encoder_b_get(void);

/* 清零计数值 */
void encoder_a_reset(void);
void encoder_b_reset(void);

#endif
