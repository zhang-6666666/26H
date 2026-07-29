/* 任务调度 — HAL 回调 + 系统节拍 + 应用层轮询 */
#include "task.h"
#include "motor.h"
#include "encoder.h"
#include "control.h"
#include "cmd_handler.h"
#include "gray.h"
#include "uartdbg.h"
#include "usart.h"
#include "tim.h"


/* ===================== 常量 ===================== */
#define SEND_INTERVAL  100   /* 调试输出周期 (ms) */
#define PID_INTERVAL   20    /* 控制计算周期 (ms) */

/* ===================== 内部状态 ===================== */
static volatile uint32_t s_tick;         /* 1ms 系统节拍（TIM3 ISR 自增） */
static uint32_t s_last_send;            /* 上次发送时刻 (tick) */
static uint32_t last_pid_tick;          /* 上次 PID 计算时刻 (tick) */


uint32_t task_tick(void)
{
    return s_tick;
}

/* ===================== HAL 弱回调覆盖 ===================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) s_tick++;
}

/* ===================== 初始化 ===================== */

void task_init(void)
{
    HAL_TIM_Base_Start_IT(&htim3);
    UartDbg_Init(&uart_dbg, &huart1, &hdma_usart1_tx, &hdma_usart1_rx);
    UartDbg_SetCmdCb(&uart_dbg, CmdHandler_Process);

    control_init();
    Encoder_Init(&encoder_left,  &htim2, 1);
    Encoder_Init(&encoder_right, &htim4, 0);



    Motor_Config cfg_l = {
        .ain1_port = GPIOB, .ain1_pin = GPIO_PIN_13,
        .ain2_port = GPIOB, .ain2_pin = GPIO_PIN_12,
        .htim = &htim1, .pwm_channel = TIM_CHANNEL_4, .pwm_period = 999,
    };
    Motor_Config cfg_r = {
        .ain1_port = GPIOB, .ain1_pin = GPIO_PIN_14,
        .ain2_port = GPIOB, .ain2_pin = GPIO_PIN_15,
        .htim = &htim1, .pwm_channel = TIM_CHANNEL_1, .pwm_period = 999,
    };

    Motor_Init(&motor_left,  &cfg_l);
    Motor_Init(&motor_right, &cfg_r);
    control_set_speed(-0.0f,-0.0f);
}

/* ===================== 主循环轮询 ===================== */

void task_poll(void)
{
    
    UartDbg_Poll(&uart_dbg);  /* 串口命令处理 */

    /* 100ms：调试输出 */
    if (s_tick - s_last_send >= SEND_INTERVAL) {
        s_last_send = s_tick;
        extern GraySensor gs;
        UartDbg_Send(&uart_dbg, "spd_a=%.1f spd_b=%.1f  pos=%d\r\n raw=%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                     encoder_left.speed_cm_s, encoder_right.speed_cm_s, Gray_Position(&gs),
                     gs.raw[0],gs.raw[1],gs.raw[2],gs.raw[3],gs.raw[4],gs.raw[5],gs.raw[6],gs.raw[7]);
    }

    /* 20ms：PID 控制 */
    if (s_tick - last_pid_tick >= PID_INTERVAL) {
        last_pid_tick = s_tick;
        control_update();
        // CmdHandler_SendVofa();  /* VOFA+ 数据流（vofa=1 时生效） */
    }
}
