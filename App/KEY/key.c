#include "key.h"
#include "stm32f1xx_hal.h"

/* 按键状态机: S0~S2 */
enum {
    S_IDLE         = 0,  /* 空闲: 等按下                                 */
    S_PRESSED      = 1,  /* 已按下: 等松开/长按                          */
    S_LONG_PRESSED = 2,  /* 长按: 等松开/重复                            */
};

typedef struct {
    GPIO_TypeDef *port;      /* GPIO 端口                                  */
    uint32_t      pin;       /* 引脚号                                    */
    uint8_t       state;     /* 当前状态 S0~S2                            */
    uint8_t       timer_count; /* 通用计数器, 各状态复用 (×20ms)          */
} KeyInfo_t;

/* 按键状态数组，位域结构体替代位掩码 */
volatile Key_State key[KEY_NUM] = {{0}};

static KeyInfo_t key_info[KEY_NUM] = {
    { KEY_KEY_1_PORT, KEY_KEY_1_PIN, S_IDLE, 0 },
    { KEY_KEY_2_PORT, KEY_KEY_2_PIN, S_IDLE, 0 },
    { KEY_KEY_3_PORT, KEY_KEY_3_PIN, S_IDLE, 0 },
};

void Key_Edge_Scan(void)
{
    for (int i = 0; i < KEY_NUM; i++) {
        KeyInfo_t *k = &key_info[i];
        /* 硬件上拉: 按下→低(0) / 松开→高(非零)                        */
        /* 取反归一化: pin_state = 1=按下 / 0=松开                     */
        uint8_t pin_state = HAL_GPIO_ReadPin(k->port, k->pin) ? 0 : 1;

        switch (k->state) {

        case S_IDLE:                                    /* 空闲：等按下     */
            if (pin_state) {
                k->state       = S_PRESSED;
                k->timer_count = 0;
                key[i].down = 1;
                key[i].hold = 1;
            }
            break;

        case S_PRESSED:                                 /* 已按下：等松开/长按 */
            if (pin_state) {
                k->timer_count++;
                if (k->timer_count >= KEY_LONG_PRESS_CNT) {
                    k->state       = S_LONG_PRESSED;
                    k->timer_count = 0;
                    key[i].long_press = 1;              /* →S2, long_press=1 */
                }
            } else {
                k->state  = S_IDLE;
                key[i].up     = 1;
                key[i].single = 1;
                key[i].down   = 0;
                key[i].hold   = 0;
            }
            break;

        case S_LONG_PRESSED:                            /* 长按：等松开/重复 */
            if (pin_state) {
                k->timer_count++;
                if (k->timer_count >= KEY_REPEAT_PRESS_CNT) {
                    k->timer_count = 0;
                    key[i].repeat = 1;                  /* 自循环, repeat=1 */
                }
            } else {
                k->state  = S_IDLE;
                key[i].up     = 1;
                key[i].down   = 0;
                key[i].hold   = 0;
                key[i].repeat = 0;
            }
            break;
        }
    }
}

void Key_Read(uint8_t i, Key_State *out)
{
    if (i >= KEY_NUM || out == NULL) return;
    *out = key[i];
    /* 清除一次性标志，hold 是持续状态不自动清除 */
    key[i].down       = 0;
    key[i].up         = 0;
    key[i].single     = 0;
    key[i].long_press = 0;
    key[i].repeat     = 0;
}
