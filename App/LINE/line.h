/* 四路巡线传感器 — 低耦合，只依赖 HAL GPIO */
#ifndef LINE_H
#define LINE_H

#include <stdint.h>

void    line_init(void);        /* GPIO 已由 CubeMX 配置，空函数 */
void    line_update(void);      /* 读 4 路传感器，更新内部状态 */
int8_t  line_position(void);    /* 返回线位置: -3(左)~+3(右)，0 居中，127=丢线 */
int8_t  line_count(void);       /* 检测到线的传感器数量 */
uint8_t line_raw(void);         /* 4 位: bit3=C15 bit2=A6 bit1=A5 bit0=A4 */

#endif
