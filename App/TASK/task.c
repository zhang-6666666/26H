/* 任务调度实现 — HAL 回调 + DMA 发送 + 电机状态机 */
#include "task.h"
#include "jy901p.h"
#include "motor.h"
#include "usart.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>

/* ===================== 内部状态 ===================== */

static volatile uint32_t s_tick;      /* 1ms 系统节拍（TIM2 ISR 自增） */
static volatile uint8_t  s_tx_busy;   /* 1 = DMA 正在发送中 */
static uint32_t s_last_send;          /* 上次 DMA 发送时刻 (tick) */

static char  s_tx_buf[64];            /* DMA 安全缓冲区（copy 后 DMA 使用） */
static uint8_t s_angle_pending;       /* s_tx_buf 中有待发数据 */

static uint32_t s_fsm_start;          /* 当前 FSM 阶段起始时刻 */
static uint8_t  s_fsm_phase;          /* 0=正转 1=刹 2=反 3=刹 */

#define SEND_INTERVAL  100            /* 100ms 发送一次 */

/* ===================== HAL 弱回调覆盖 ===================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) s_tick++;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) s_tx_busy = 0;
}

/* ===================== 公开 API ===================== */

void task_init(void)
{
    s_tick          = 0;
    s_tx_busy       = 0;
    s_angle_pending = 0;
    s_last_send     = 0;
    s_fsm_phase     = 0;
    s_fsm_start     = 0;
    HAL_TIM_Base_Start_IT(&htim2);
}

uint32_t task_tick(void)
{
    return s_tick;
}

void task_poll(void)
{
    /* ① 角度就绪 → 立即从 SDK 取走字符串，拷贝到 DMA 安全缓冲 */
    if (jy901p_angle_ready()) {
        snprintf(s_tx_buf, sizeof(s_tx_buf), "%s\r\n", jy901p_angle_str());
        s_angle_pending = 1;
    }

    /* ② 每 100ms：有待发数据 + DMA 空闲 → 启动 DMA 发送 */
    if (s_angle_pending
        && !s_tx_busy
        && (s_tick - s_last_send >= SEND_INTERVAL))
    {
        s_angle_pending = 0;
        s_tx_busy       = 1;
        s_last_send     = s_tick;
        HAL_UART_Transmit_DMA(&huart1,
                              (uint8_t *)s_tx_buf, strlen(s_tx_buf));
    }

    /* ③ 电机测试状态机（基于 s_tick 计时，永不阻塞） */
    uint32_t elapsed = s_tick - s_fsm_start;
    switch (s_fsm_phase) {
        case 0:   /* 正转 2s */
            motor_a_run(200);
            motor_b_run(200);
            if (elapsed >= 2000) { s_fsm_phase = 1; s_fsm_start = s_tick; }
            break;
        case 1:   /* 刹车 1s */
            motor_a_brake();
            motor_b_brake();
            if (elapsed >= 1000) { s_fsm_phase = 2; s_fsm_start = s_tick; }
            break;
        case 2:   /* 反转 2s */
            motor_a_run(-200);
            motor_b_run(-200);
            if (elapsed >= 2000) { s_fsm_phase = 3; s_fsm_start = s_tick; }
            break;
        case 3:   /* 刹车 1s */
            motor_a_brake();
            motor_b_brake();
            if (elapsed >= 1000) { s_fsm_phase = 0; s_fsm_start = s_tick; }
            break;
    }
}
