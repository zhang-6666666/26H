/* 任务调度系统 — 按键选择 + 启动/停止 + 可扩展 */
#ifndef QUESTION_H
#define QUESTION_H

#include <stdint.h>

/* 任务定义 */
typedef struct {
    uint8_t   id;
    const char *name;
    void     (*start)(void);         /* 启动回调 */
    uint8_t  (*update)(void);        /* 每帧调用，返回 1=完成 */
    void     (*stop)(void);          /* 停止回调 */
} TaskDef;

/* 最大任务数 */
#define TASK_MAX  8

void Question_Init(void);
void Question_Update(void);          /* 50ms 调用一次 */

uint8_t Question_Running(void);      /* 当前运行任务 ID，0xFF=无 */
uint8_t Question_Selected(void);     /* 当前选中任务 ID */
const char *Question_Name(uint8_t id); /* 任务名 */

#endif
