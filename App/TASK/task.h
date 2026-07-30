/* 任务调度 */
#ifndef TASK_H
#define TASK_H

void     task_init(void);
void     task_poll(void);

#include <stdint.h>
extern volatile uint32_t s_tick;
extern volatile uint32_t s_task_tick;
extern volatile uint8_t  s_task_timer_on;

#endif
