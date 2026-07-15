/* 四路巡线传感器 — 加权位置计算 */
#include "line.h"
#include "main.h"

/* 传感器 GPIO */
#define LINE_C15_PORT  GPIOC
#define LINE_C15_PIN   GPIO_PIN_15
#define LINE_A6_PORT   GPIOA
#define LINE_A6_PIN    GPIO_PIN_6
#define LINE_A5_PORT   GPIOA
#define LINE_A5_PIN    GPIO_PIN_5
#define LINE_A4_PORT   GPIOA
#define LINE_A4_PIN    GPIO_PIN_4

/* 物理位置权重: 左=-, 右=+ */
#define W_C15  (-3)
#define W_A6   (-1)
#define W_A5   (+1)
#define W_A4   (+3)

static uint8_t s_c15, s_a6, s_a5, s_a4;  /* 1=看到线，0=没看到 */

void line_init(void) { }

void line_update(void)
{
    /* 黑线=低电平(0)，取反为 1 */
    s_c15 = !HAL_GPIO_ReadPin(LINE_C15_PORT, LINE_C15_PIN);
    s_a6  = !HAL_GPIO_ReadPin(LINE_A6_PORT,  LINE_A6_PIN);
    s_a5  = !HAL_GPIO_ReadPin(LINE_A5_PORT,  LINE_A5_PIN);
    s_a4  = !HAL_GPIO_ReadPin(LINE_A4_PORT,  LINE_A4_PIN);
}

int8_t line_position(void)
{
    int8_t sum = (int8_t)(s_a4 * W_A4 + s_a5 * W_A5 + s_a6 * W_A6 + s_c15 * W_C15);
    int8_t cnt = (int8_t)(s_a4 + s_a5 + s_a6 + s_c15);

    if (cnt == 0) return 127;  /* 丢线 */
    return sum / cnt;
}

int8_t line_count(void)
{
    return (int8_t)(s_a4 + s_a5 + s_a6 + s_c15);
}

uint8_t line_raw(void)
{
    return (s_c15 << 3) | (s_a6 << 2) | (s_a5 << 1) | s_a4;
}
