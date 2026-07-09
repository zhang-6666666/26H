/* 任务调度 — HAL 回调 + PID 控制 + DMA 发送 */
#include "task.h"
#include "wit_c_sdk.h"
#include "jy901p.h"
#include "motor.h"
#include "encoder.h"
#include "control.h"
#include "usart.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>

/* ===================== 内部状态 ===================== */
static volatile uint32_t s_tick;      /* 1ms 系统节拍（TIM3 ISR 自增） */
static volatile uint8_t  s_tx_busy;   /* 1 = DMA 正在发送中 */
static uint32_t s_last_send;          /* 上次 DMA 发送时刻 (tick) */
static uint32_t last_pid_tick;      /* 上次 PID 计算时刻 (tick) */

static char  s_tx_buf[128];           /* DMA 安全缓冲区 */

#define SEND_INTERVAL  100            /* 100ms 发送一次 */
#define PID_INTERVAL   50          /* 50ms PID 计算一次 */

/* ===================== HAL 弱回调覆盖 ===================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) s_tick++;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) s_tx_busy = 0;
}

/* ===================== 公开 API ===================== */


void task_init(void)
{
    HAL_TIM_Base_Start_IT(&htim3);
    jy901p_init(&huart2, &hdma_usart2_rx);

    control_init();
    Encoder_Init(&left_encoder,  &htim2, 1);
    Encoder_Init(&right_encoder, &htim4, 0);
    /* 传感器配置：只输出角度，提高数据刷新率 */
    WitSetContent(RSW_ANGLE);
    jy901p_set_6axis();   /* 切换到6轴算法（仅首次需要，保存后永久生效） */
    /* 等待传感器稳定（非阻塞，持续处理串口数据） */
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() - t < 3000) 
    {
        jy901p_poll();
    }
    jy901p_zero();  /* 稳定后归零 */    
    printf("JY901P init done\r\n");
    motor_init(&htim1);
    control_set_speed(-20.0f,-20.0f);
    control_set_yaw(0.0f);
}



void task_poll(void)
{

    //陀螺仪轮询获取最新角度
    jy901p_poll();

    /* ① 每 100ms：打印角度 → DMA 发送 */
    if (!s_tx_busy && (s_tick - s_last_send >= SEND_INTERVAL))
    {
        s_last_send = s_tick;
        snprintf(s_tx_buf, sizeof(s_tx_buf),
                 "Yaw=%.1f enc_a=%ld enc_b=%ld speed_a=%.1f speed_b=%.1f\r\n",
                  angle_y,
                 (long)left_encoder.total_count, (long)right_encoder.total_count,
                 (float)left_encoder.speed_cm_s, (float)right_encoder.speed_cm_s);
        s_tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)s_tx_buf, strlen(s_tx_buf));
    }

    /* ② 每 50ms：PID 控制 */
    if (s_tick - last_pid_tick >= PID_INTERVAL)
    {
        last_pid_tick = s_tick;
        control_update(PID_INTERVAL * 0.001f);
    }

}

