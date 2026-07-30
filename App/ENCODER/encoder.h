/* 编码器驱动 — 配置结构体 + 句柄模式 */
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

#define ENCODER_PPR          1060    /* 13线/相, 20倍减速比, 4倍频 */
#define WHEEL_DIAMETER_CM     4.8f   /* 轮径 (cm) */
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * 3.14159265f)
#define SAMPLING_TIME_S        0.050f /* 采样周期 (s) */

typedef struct {
    TIM_HandleTypeDef *htim;
    uint8_t  reverse;
    int16_t  count;
    int32_t  total_count;
    float    speed_cm_s;
} Encoder;

void Encoder_Init(Encoder *enc, TIM_HandleTypeDef *htim, uint8_t reverse);
void Encoder_Reset(Encoder *enc);
void Encoder_Update(Encoder *enc);

extern Encoder encoder_left;
extern Encoder encoder_right;

#endif
