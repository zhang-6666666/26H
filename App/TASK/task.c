/* 任务调度 — HAL 回调 + 系统节拍 + 应用层轮询 */
#include "task.h"
#include "cmd_handler.h"
#include "control.h"
#include "encoder.h"
#include "gray.h"
#include "i2c.h"
#include "key.h"
#include "motor.h"
#include "oled.h"
#include "question.h"
#include "tim.h"
#include "uartdbg.h"
#include "usart.h"


/* ===================== 常量 ===================== */
#define SEND_INTERVAL   100   /* 调试输出周期 (ms) */
#define PID_INTERVAL    50    /* 控制计算周期 (ms) */
#define KEY_INTERVAL    10    /* 按键扫描周期 (ms) */
#define OLED_INTERVAL   200   /* OLED 刷新周期 (ms) */

/* ===================== 内部状态 ===================== */
static volatile uint32_t s_tick;          /* 1ms 系统节拍（TIM3 ISR 自增） */
static volatile uint32_t s_sec;           /* 秒计数（TIM3 ISR，1000ms 进位） */
static uint32_t s_ms_acc;                 /* ISR 内部毫秒累加器 */
static uint32_t s_last_send, s_last_pid, s_last_key, s_last_oled;

uint32_t task_tick(void) { return s_tick; }
uint32_t task_sec(void)  { return s_sec;  }

/* ===================== ISR ===================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        s_tick++;
        if (++s_ms_acc >= 1000) { s_ms_acc = 0; s_sec++; }
        Key_Tick();  /* 按键扫描: 1ms tick → 内部 20 分频 = 20ms 周期 */
    }
}

/* ===================== 初始化 ===================== */
void task_init(void)
{
    HAL_TIM_Base_Start_IT(&htim3);
    UartDbg_Init(&uart_dbg, &huart1, &hdma_usart1_tx, &hdma_usart1_rx);
    UartDbg_SetCmdCb(&uart_dbg, CmdHandler_Process);

    oled_init(&hi2c1);

    Question_Init();
    control_init(CTRL_STOP, 0, 0);    /* 初始停止态 */
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
}

/* ===================== 主循环轮询 ===================== */
void task_poll(void)
{
    UartDbg_Poll(&uart_dbg);

    /* 灰度 + 串口 */
    if (s_tick - s_last_send >= SEND_INTERVAL) {
        s_last_send = s_tick;
        extern GraySensor gs;
        Gray_Update(&gs);
        uint8_t d = gs.digital;
        UartDbg_Send(&uart_dbg,
            "G[%d%d%d%d|%d%d%d%d] pos=%d\r\n",
            (d>>0)&1,(d>>1)&1,(d>>2)&1,(d>>3)&1,
            (d>>4)&1,(d>>5)&1,(d>>6)&1,(d>>7)&1,
            Gray_Position(&gs));
    }

    /* 按键 */
    if (s_tick - s_last_key >= KEY_INTERVAL) {
        s_last_key = s_tick;
        Key_Tick();
    }

    /* 任务调度 + PID 控制 */
    if (s_tick - s_last_pid >= PID_INTERVAL) {
        s_last_pid = s_tick;
        Question_Update();
        control_update();
    }

    /* OLED */
    if (s_tick - s_last_oled >= OLED_INTERVAL) {
        s_last_oled = s_tick;

        uint8_t run = Question_Running();
        uint8_t sel = Question_Selected();

        /* 第一行：状态 + 任务名 */
        oled_set_cursor(0, 0);
        if (run != 0xFF) {
            oled_print("R:");
            oled_print(Question_Name(run));
        } else {
            oled_print("S:");
            oled_print(Question_Name(sel));
        }

        /* 第二行：运行秒数 */
        oled_set_cursor(1, 0);
        oled_print("T:");
        char num[6];
        uint32_t v = s_tick / 1000;
        uint8_t i = sizeof(num) - 1;
        num[i--] = '\0';
        if (v == 0) num[i--] = '0';
        else while (v) { num[i--] = '0' + (v % 10); v /= 10; }
        oled_print(&num[i + 1]);

        oled_show();
    }
}
