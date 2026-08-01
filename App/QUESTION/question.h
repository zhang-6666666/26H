/* 任务调度系统 — 按键选择 + 启动/停止 + 可扩展 */
#ifndef QUESTION_H
#define QUESTION_H

#include <stdint.h>

/* 任务定义 */
typedef struct {
    uint8_t   id;
    const char *name;
    void     (*run)(void);            /* 任务回调，内部自己处理启停 */
} TaskDef;


void Question_Init(void);
void Question_Update(void);

uint8_t   Question_Running(void);       /* 当前运行任务 ID，0xFF=无 */
uint8_t   Question_Selected(void);      /* 当前选中任务 ID */
const char *Question_Name(uint8_t id);

#endif
