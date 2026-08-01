/* 任务调度系统 */
#include "question.h"
#include "control.h"
#include "key.h"
#include "task.h"

/* 内部状态封装 */
typedef struct {
    const TaskDef *tasks;   /* 任务列表（驻留 Flash） */
    uint8_t        count;   /* 任务数量 */
    uint8_t        selected;/* 当前选中索引 */
} Question;

static Question s_q;

static void Task_Line(void)
{
    control_init(CTRL_LINE, 40.0f, 40.0f);
}

/* 任务列表（扩展只需在此增项） */
static const TaskDef s_tasks[] =
{
    {0, "Line", Task_Line},
};

void Question_Init(void)
{
    s_q.tasks    = s_tasks;
    s_q.count    = sizeof(s_tasks) / sizeof(s_tasks[0]);
    s_q.selected = 0;
}

uint8_t Question_Running(void)
{
    return (control_get_mode() == CTRL_STOP) ? 0xFF : s_q.selected;
}

uint8_t Question_Selected(void)
{
    return s_q.tasks[s_q.selected].id;
}

const char *Question_Name(uint8_t id)
{
    return (id < s_q.count) ? s_q.tasks[id].name : "?";
}

void Question_Update(void)
{
    /* 停止计时：自动停车或手动停止后 control 已切 STOP */
    if (s_task_timer_on && control_get_mode() == CTRL_STOP)
        s_task_timer_on = 0;

    if (control_get_mode() == CTRL_STOP) {
        /* 空闲态 */
        if (key[0].single) {
            key[0].single = 0;
            s_q.selected = (s_q.selected + s_q.count - 1) % s_q.count;
        }
        if (key[1].single) {
            key[1].single = 0;
            s_q.selected = (s_q.selected + 1) % s_q.count;
        }
        if (key[2].single) {
            key[2].single = 0;
            if (s_q.tasks[s_q.selected].run) s_q.tasks[s_q.selected].run();
             s_task_tick    = 0;
            s_task_timer_on = 1;
        }
    } else {
        /* 运行态 */
        if (key[2].single) {
            key[2].single = 0;
            s_task_timer_on = 0;
            control_init(CTRL_STOP, 0, 0);
        }
    }
}


