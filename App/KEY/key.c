#include "key.h"

#include "stm32f1xx_hal.h"

/* 按键状态机: S0~S2                                               */
enum {
    S_IDLE         = 0,  /* 空闲: 等按下                           */
    S_PRESSED      = 1,  /* 已按下: 等松开/长按                    */
    S_LONG_PRESSED = 2,  /* 长按: 等松开/重复                      */
};

typedef struct {
    GPIO_TypeDef *port;      /* GPIO 端口                            */
    uint32_t   pin;          /* 引脚号                               */
    uint8_t    state;        /* 当前状态 S0~S2                       */
    uint8_t    timer_count;  /* 通用计数器, 各状态复用 (×20ms)       */
} KeyInfo_t;

/* 按键标志数组, 每个键 1 字节按位使用 (见 key.h 位掩码)            */
volatile uint8_t key_flag[KEY_NUM] = {0};

static KeyInfo_t key_info[KEY_NUM] = {
    { KEY_KEY_1_PORT, KEY_KEY_1_PIN, S_IDLE, 0 },
    { KEY_KEY_2_PORT, KEY_KEY_2_PIN, S_IDLE, 0 },
    { KEY_KEY_3_PORT, KEY_KEY_3_PIN, S_IDLE, 0 },
};

void Key_Edge_Scan(void)
{
    for (int i = 0; i < KEY_NUM; i++) {
        KeyInfo_t *k = &key_info[i];
        /* 硬件上拉: 按下→低(0) / 松开→高(非零)                          */
        /* 取反归一化: pin_state = 1=按下 / 0=松开                       */
        uint8_t   pin_state = HAL_GPIO_ReadPin(k->port, k->pin) ? 0 : 1;

        switch (k->state) {

        case S_IDLE:                                    /* 空闲：等按下       */
            if (pin_state) {
                k->state       = S_PRESSED;
                k->timer_count = 0;
                key_flag[i]   |= (KEY_FLAG_DOWN | KEY_FLAG_HOLD);
            }
            break;

        case S_PRESSED:                                 /* 已按下：等松开/长按 */
            if (pin_state) {
                k->timer_count++;
                if (k->timer_count >= KEY_LONG_PRESS_CNT) {
                    k->state       = S_LONG_PRESSED;
                    k->timer_count = 0;
                    key_flag[i]   |= KEY_FLAG_LONG;     /* →S2, LONG=1     */
                }
            } else {
                k->state     = S_IDLE;
                key_flag[i] |= (KEY_FLAG_UP | KEY_FLAG_SINGLE);
                key_flag[i] &= ~(KEY_FLAG_DOWN | KEY_FLAG_HOLD);
            }
            break;

        case S_LONG_PRESSED:                            /* 长按：等松开/重复   */
            if (pin_state) {
                k->timer_count++;
                if (k->timer_count >= KEY_REPEAT_PRESS_CNT) {
                    k->timer_count = 0;
                    key_flag[i]   |= KEY_FLAG_REPEAT;   /* 自循环, REPEAT=1 */
                }
            } else {
                k->state     = S_IDLE;
                key_flag[i] |= KEY_FLAG_UP;
                key_flag[i] &= ~(KEY_FLAG_DOWN | KEY_FLAG_HOLD | KEY_FLAG_REPEAT);
            }
            break;
        }
    }
}

uint8_t Key_Check(uint8_t Flag)
{
    for (int i = 0; i < KEY_NUM; i++) {
        if (key_flag[i] & Flag) {
            if (Flag != KEY_FLAG_HOLD) {
                key_flag[i] &= ~Flag;
            }
            return 1;
        }
    }
    return 0;
}

void Key_Tick(void)
{
    static uint8_t Count;

    Count++;
    if (Count >= 20) {
        Count = 0;
        Key_Edge_Scan();
    }
}
