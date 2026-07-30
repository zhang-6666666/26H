/* 任务调度系统 */
#include "question.h"
#include "control.h"
#include "key.h"
#include "task.h"

static void Task_Line(void)
{
    control_init(CTRL_LINE, 40.0f, 40.0f);
}

//设定任务列表，id=0~7，name为显示名称，run为启动回调
static const TaskDef s_tasks[] =
{
    {0, "Line", Task_Line},
};

#define TASK_COUNT  (sizeof(s_tasks) / sizeof(s_tasks[0]))


static uint8_t s_selected;

uint8_t Question_Running(void)  { return (control_get_mode() == CTRL_STOP) ? 0xFF : s_selected; }
uint8_t Question_Selected(void) { return s_tasks[s_selected].id; }
const char *Question_Name(uint8_t id) { return (id < TASK_COUNT) ? s_tasks[id].name : "?"; }
void Question_Init(void) { s_selected = 0; }

void Question_Update(void)
{
    /* 停止计时：自动停车或手动停止后 control 已切 STOP */
    if (s_task_timer_on && control_get_mode() == CTRL_STOP)
        s_task_timer_on = 0;

    if (control_get_mode() == CTRL_STOP) {
        /* 空闲态 */
        if (key_flag[0] & KEY_FLAG_SINGLE) {
            key_flag[0] &= ~KEY_FLAG_SINGLE;
            s_selected = (s_selected + TASK_COUNT - 1) % TASK_COUNT;
        }
        if (key_flag[1] & KEY_FLAG_SINGLE) {
            key_flag[1] &= ~KEY_FLAG_SINGLE;
            s_selected = (s_selected + 1) % TASK_COUNT;
        }
        if (key_flag[2] & KEY_FLAG_SINGLE) {
            key_flag[2] &= ~KEY_FLAG_SINGLE;
            if (s_tasks[s_selected].run) s_tasks[s_selected].run();
            s_task_tick    = 0;
            s_task_timer_on = 1;
        }
    } else {
        /* 运行态 */
        if (key_flag[2] & KEY_FLAG_SINGLE) {
            key_flag[2] &= ~KEY_FLAG_SINGLE;
            s_task_timer_on = 0;
            control_init(CTRL_STOP, 0, 0);
        }
    }
}


