#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx_hal.h"

/* ── 按键参数宏 ─────────────────────────────────────────── */
#define KEY_SCAN_MS           10    /* 扫描周期 (ms)                         */
#define KEY_LONG_PRESS_MS    2000   /* 长按判定阈值 (ms)                     */
#define KEY_REPEAT_PRESS_MS   200   /* 长按重复间隔 (ms)                     */

/* 转换为扫描次数 */
#define KEY_LONG_PRESS_CNT    (KEY_LONG_PRESS_MS   / KEY_SCAN_MS)
#define KEY_REPEAT_PRESS_CNT  (KEY_REPEAT_PRESS_MS / KEY_SCAN_MS)

/* ── 按键引脚定义（根据实际硬件修改） ────────────────────────── */
#define KEY_KEY_1_PORT      GPIOA
#define KEY_KEY_1_PIN       GPIO_PIN_4
#define KEY_KEY_2_PORT      GPIOA
#define KEY_KEY_2_PIN       GPIO_PIN_5
#define KEY_KEY_3_PORT      GPIOA
#define KEY_KEY_3_PIN       GPIO_PIN_12
#define KEY_NUM              3      /* 按键数量                             */

/* ── 按键状态（位域结构体，替代位掩码宏） ──────────────────────── */
typedef struct {
    uint8_t hold       : 1;   /* Bit 0: 按住中                             */
    uint8_t down       : 1;   /* Bit 1: 按下瞬间                           */
    uint8_t up         : 1;   /* Bit 2: 释放瞬间                           */
    uint8_t single     : 1;   /* Bit 3: 单击                               */
    uint8_t long_press : 1;   /* Bit 5: 长按                               */
    uint8_t repeat     : 1;   /* Bit 6: 重复                               */
} Key_State;

/* ── 按键状态数组 ──────────────────────────────────────────── */
extern volatile Key_State key[KEY_NUM];

/* ── API ──────────────────────────────────────────────────── */
void Key_Edge_Scan(void);
void Key_Read(uint8_t i, Key_State *out);   /* 读取并清除一次性标志       */

#endif /* __KEY_H */
