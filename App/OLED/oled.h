/* SSD1306 OLED 128×64 I2C 驱动 — 低耦合，只依赖 HAL 库 */
#ifndef OLED_H
#define OLED_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 初始化：传入 I2C 句柄，完成 SSD1306 寄存器配置序列 */
void oled_init(I2C_HandleTypeDef *hi2c);

/* 快捷方法：清屏 → 显示一行 → 刷新。line: 0~7 */
void oled_show_string(uint8_t line, const char *str);

/* 分步方法 */
void oled_clear(void);                          /* 清显存（不刷新） */
void oled_set_cursor(uint8_t x, uint8_t y);     /* 打印位置 x:0~127, y:行号0~7 */
void oled_print(const char *str);               /* 当前位置打印 ASCII 字符串 */
void oled_show(void);                           /* 全屏刷新到 OLED */

#endif
