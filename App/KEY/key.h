#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx_hal.h"

/* ── 按键参数宏 ─────────────────────────────────────────── */
#define KEY_SCAN_MS           20    /* 扫描周期 (ms)，与 Key_Tick 内 Count 阈值一致  */
#define KEY_LONG_PRESS_MS    2000   /* 长按判定阈值 (ms)                             */
#define KEY_REPEAT_PRESS_MS   200   /* 长按重复间隔 (ms)                             */

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
#define KEY_NUM              3      /* 按键数量                       */

/* ── 按键标志位定义（位掩码宏定义） ──────────────────────────────────── */
#define KEY_FLAG_HOLD    0x01   /* Bit 0: 按住中                   */
#define KEY_FLAG_DOWN    0x02   /* Bit 1: 按下瞬间                 */
#define KEY_FLAG_UP      0x04   /* Bit 2: 释放瞬间                 */
#define KEY_FLAG_SINGLE  0x08   /* Bit 3: 单击                     */
#define KEY_FLAG_LONG    0x20   /* Bit 5: 长按                     */
#define KEY_FLAG_REPEAT  0x40   /* Bit 6: 重复                     */

/* ── 按键标志数组 (每个键一个字节, 按位使用) ────────── */
extern volatile uint8_t key_flag[KEY_NUM];

void Key_Edge_Scan(void);
uint8_t Key_Check(uint8_t Flag);

#endif /* __KEY_H */