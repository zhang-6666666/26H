/* 任务调度 — 1ms 系统节拍 + 多任务轮询 */
#ifndef TASK_H
#define TASK_H

#include <stdint.h>

void     task_init(void);
void     task_poll(void);
uint32_t task_tick(void);

#endif
