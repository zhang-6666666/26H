/* 任务调度系统 */
#include "question.h"
#include "control.h"
#include "key.h"
#include "motor.h"

/* ==================== 任务实现 ==================== */
/* 每个任务只需实现 start / update / stop 三个回调            */

/* ── 任务 0：巡线 ── */
static void t0_start(void) { control_init(CTRL_LINE, 30.0f, 30.0f); }
static uint8_t t0_update(void) {
    /* 停车线 → 自动完成（control_update 内部切到 CTRL_STOP）*/
    return (control_get_mode() == CTRL_STOP) ? 1 : 0;
}
static void t0_stop(void) { control_init(CTRL_STOP, 0, 0); }

/* ── 任务 1：纯速度 ── */
static void t1_start(void) { control_init(CTRL_SPEED, 30.0f, 30.0f); }
static void t1_stop(void)  { control_init(CTRL_STOP, 0, 0); }

/* ── 任务 2~4：预留 ── */
static void t_stub_start(void) {}
static uint8_t t_stub_update(void) { return 1; }   /* 立即完成 */
static void t_stub_stop(void) {}

/* ==================== 任务表（扩展只需加行） ==================== */
static const TaskDef s_tasks[] = {
    { 0, "Line",  t0_start,  t0_update,  t0_stop  },
    { 1, "Speed", t1_start,  NULL,       t1_stop  },
    { 2, "Resv2", t_stub_start, t_stub_update, t_stub_stop },
    { 3, "Resv3", t_stub_start, t_stub_update, t_stub_stop },
    { 4, "Resv4", t_stub_start, t_stub_update, t_stub_stop },
};
#define TASK_COUNT  (sizeof(s_tasks) / sizeof(s_tasks[0]))

/* ==================== 内部状态 ==================== */
static uint8_t s_selected;        /* 当前选中的任务索引 */
static int8_t  s_running = -1;    /* 正在运行的任务索引，-1=无 */
static uint8_t s_key_debounce;    /* 按键去抖 */

/* ==================== API ==================== */
uint8_t Question_Running(void) { return (s_running < 0) ? 0xFF : s_tasks[s_running].id; }
uint8_t Question_Selected(void)  { return s_tasks[s_selected].id; }
const char *Question_Name(uint8_t id) { return (id < TASK_COUNT) ? s_tasks[id].name : "?"; }

void Question_Init(void)
{
    s_selected = 0;    /* 默认选中任务 0 */
    s_running  = -1;
}

void Question_Update(void)
{
    uint8_t next = 0, prev = 0, trig = 0;

    /* 读取按键（一次性事件） */
    if (key_flag[0] & KEY_FLAG_SINGLE) { key_flag[0] &= ~KEY_FLAG_SINGLE; prev = 1; }
    if (key_flag[1] & KEY_FLAG_SINGLE) { key_flag[1] &= ~KEY_FLAG_SINGLE; next = 1; }
    if (key_flag[2] & KEY_FLAG_SINGLE) { key_flag[2] &= ~KEY_FLAG_SINGLE; trig = 1; }

    if (s_running < 0) {
        /* ── 空闲态：选任务 ── */
        if (prev) s_selected = (s_selected + TASK_COUNT - 1) % TASK_COUNT;
        if (next) s_selected = (s_selected + 1) % TASK_COUNT;

        if (trig) {
            /* 启动任务 */
            const TaskDef *t = &s_tasks[s_selected];
            if (t->start) t->start();
            s_running = (int8_t)s_selected;
        }
    } else {
        /* ── 运行态 ── */
        if (trig) {
            /* 手动停止 */
            const TaskDef *t = &s_tasks[s_running];
            if (t->stop) t->stop();
            s_running = -1;
            return;
        }

        /* 检查任务是否自行完成 */
        const TaskDef *t = &s_tasks[s_running];
        if (t->update && t->update()) {
            if (t->stop) t->stop();
            s_running = -1;
        }
    }
}
