/* 8 路灰度传感器驱动 — ADC 多路复用读取 + 二值化 + 加权位置 */
#ifndef GRAY_H
#define GRAY_H

#include <stdint.h>

#define GRAY_WHITE_DEFAULT  1800
#define GRAY_BLACK_DEFAULT   100

typedef struct {
    uint16_t raw[8];       /* 原始 ADC 值 */
    uint8_t  digital;      /* 二值化结果（1=黑线，0=白色）*/
} GraySensor;

void    Gray_Init(GraySensor *gs);
void    Gray_Update(GraySensor *gs);            /* 读 8 路 ADC + 二值化 */
int8_t  Gray_Position(const GraySensor *gs);    /* -6~+6，0 居中，127=丢线 */
uint8_t Gray_Raw(const GraySensor *gs);         /* digital 原值 */

extern GraySensor gs;

#endif
