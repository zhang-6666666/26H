/* 任务调度 — 1ms 节拍 + DMA 发送 + 电机 FSM */
#ifndef TASK_H
#define TASK_H

#include <stdint.h>

/* 初始化：启动 1ms 定时器、复位状态 */
void task_init(void);

/* 主循环每轮调用一次：DMA 发送管理 + 电机测试状态机 */
void task_poll(void);

/* 获取系统节拍（供其他模块使用） */
uint32_t task_tick(void);

#endif
