/* 编码器 — 硬件 TIM 正交解码，TI1+TI2 4 倍频 */
#include "encoder.h"
#include "tim.h"

void encoder_init(void)
{
    /* 启动硬件编码器计数（纯后台，无中断） */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

/* 直接读 CNT 寄存器，16bit 转 int32_t（编码器模式 CNT 自动增减） */
int32_t encoder_a_get(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

int32_t encoder_b_get(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

void encoder_a_reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

void encoder_b_reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}
