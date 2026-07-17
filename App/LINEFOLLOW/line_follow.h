/* 巡线策略引擎 — 重心法 + 直角弯 + 丢线处理 */
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

typedef struct {
    float   steer;      /* 转向量（正=右转，负=左转）*/
    uint8_t on_line;    /* 0=停车 1=在线 2=直角弯中 */
    uint8_t last_turn;  /* 上次转向方向（0=左 1=右）*/
} LineFollow_Result;

LineFollow_Result LineFollow_Update(void);

#endif
