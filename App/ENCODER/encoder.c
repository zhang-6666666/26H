#include "encoder.h"
#include "tim.h"

Encoder encoder_left;
Encoder encoder_right;

void Encoder_Reset(Encoder *enc)
{
    enc->count       = 0;
    enc->total_count = 0;
    enc->speed_cm_s  = 0.0f;
    __HAL_TIM_SetCounter(enc->htim, 0);
}

void Encoder_Init(Encoder *enc, TIM_HandleTypeDef *htim, uint8_t reverse)
{
    enc->htim     = htim;
    enc->reverse  = reverse;
    enc->count    = 0;
    enc->total_count = 0;
    enc->speed_cm_s = 0.0f;

    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SetCounter(enc->htim, 0);
}

void Encoder_Update(Encoder *enc)
{
    enc->count = (int16_t)__HAL_TIM_GetCounter(enc->htim);
    enc->count = enc->reverse ? -(enc->count) : enc->count;
    __HAL_TIM_SetCounter(enc->htim, 0);

    enc->total_count += enc->count;

    enc->speed_cm_s = (float)enc->count / ENCODER_PPR
                    * WHEEL_CIRCUMFERENCE_CM / SAMPLING_TIME_S;
}
