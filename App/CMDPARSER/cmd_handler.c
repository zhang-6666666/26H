/* 串口命令解析 + VOFA+ JustFloat 输出 */
#include "cmd_handler.h"
#include "control.h"
#include "encoder.h"
#include "jy901p.h"
#include "uartdbg.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

volatile uint8_t g_vofa_enable = 0;

/* ===================== 命令执行 ===================== */

static void cmd_ping(const char *args)
{
    (void)args;
    UartDbg_Send(&uart_dbg, "pong\r\n");
}

static void cmd_help(const char *args)
{
    (void)args;
    UartDbg_Send(&uart_dbg,
        "spd=left,right     set speed targets (cm/s)\r\n"
        "yaw=angle          set yaw target (°)\r\n"
        "pida=kp,ki,kd      set motor A speed PID\r\n"
        "pidb=kp,ki,kd      set motor B speed PID\r\n"
        "pindy=kp,ki,kd     set yaw PID\r\n"
        "vofa=0/1           disable/enable VOFA+ data stream\r\n"
        "mode=stop|speed|yaw|line  control mode\r\n"
        "ping               check connection\r\n"
        "help               this list\r\n");
}

static void cmd_spd(const char *args)
{
    float a, b;
    if (sscanf(args, "%f,%f", &a, &b) == 2) {
        control_set_speed(a, b);
        UartDbg_Send(&uart_dbg, "spd=%.1f,%.1f\r\n", a, b);
    } else {
        UartDbg_Send(&uart_dbg, "usage: spd=left,right\r\n");
    }
}

static void cmd_yaw(const char *args)
{
    float a;
    if (sscanf(args, "%f", &a) == 1) {
        control_set_yaw(a);
        UartDbg_Send(&uart_dbg, "yaw=%.1f\r\n", a);
    } else {
        UartDbg_Send(&uart_dbg, "usage: yaw=angle\r\n");
    }
}

static void cmd_pid_set(const char *args, float kp, float ki, float kd,
                        void (*setter)(float, float, float), const char *name)
{
    if (sscanf(args, "%f,%f,%f", &kp, &ki, &kd) == 3) {
        setter(kp, ki, kd);
        UartDbg_Send(&uart_dbg, "%s=%.2f,%.2f,%.2f\r\n", name, kp, ki, kd);
    } else {
        UartDbg_Send(&uart_dbg, "usage: %s=kp,ki,kd\r\n", name);
    }
}

static void cmd_pida(const char *args)
{
    cmd_pid_set(args, 0, 0, 0, control_set_pid_speed_a, "pida");
}

static void cmd_pidb(const char *args)
{
    cmd_pid_set(args, 0, 0, 0, control_set_pid_speed_b, "pidb");
}

static void cmd_pindy(const char *args)
{
    cmd_pid_set(args, 0, 0, 0, control_set_pid_yaw, "pindy");
}

static void cmd_vofa(const char *args)
{
    int v;
    if (sscanf(args, "%d", &v) == 1) {
        g_vofa_enable = (v != 0);
        UartDbg_Send(&uart_dbg, "vofa=%d\r\n", g_vofa_enable);
    } else {
        UartDbg_Send(&uart_dbg, "usage: vofa=0 or vofa=1\r\n");
    }
}

static void cmd_mode(const char *args)
{
    if (strcmp(args, "stop") == 0)  { control_set_mode(CTRL_STOP);  UartDbg_Send(&uart_dbg, "mode=stop\r\n");  return; }
    if (strcmp(args, "speed") == 0) { control_set_mode(CTRL_SPEED); UartDbg_Send(&uart_dbg, "mode=speed\r\n"); return; }
    if (strcmp(args, "yaw") == 0)   { control_set_mode(CTRL_YAW);   UartDbg_Send(&uart_dbg, "mode=yaw\r\n");   return; }
    if (strcmp(args, "line") == 0)  { control_set_mode(CTRL_LINE);  UartDbg_Send(&uart_dbg, "mode=line\r\n");  return; }
    UartDbg_Send(&uart_dbg, "usage: mode=stop|speed|yaw|line\r\n");
}

/* ===================== 命令表 ===================== */

typedef struct {
    const char *name;
    void (*handler)(const char *args);
} CmdEntry;

static const CmdEntry s_cmds[] = {
    {"ping",  cmd_ping},
    {"help",  cmd_help},
    {"spd",   cmd_spd},
    {"yaw",   cmd_yaw},
    {"pida",  cmd_pida},
    {"pidb",  cmd_pidb},
    {"pindy", cmd_pindy},
    {"vofa",  cmd_vofa},
    {"mode",  cmd_mode},
};

void CmdHandler_Process(const char *line)
{
    /* 跳过前导空白 */
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    /* 按 '=' 分隔命令名和参数 */
    const char *eq = strchr(line, '=');
    const char *args = (eq) ? eq + 1 : "";

    /* 计算命令名长度并匹配 */
    size_t name_len = (eq) ? (size_t)(eq - line) : strlen(line);

    for (size_t i = 0; i < sizeof(s_cmds) / sizeof(s_cmds[0]); i++) {
        if (strlen(s_cmds[i].name) == name_len &&
            strncmp(s_cmds[i].name, line, name_len) == 0) {
            s_cmds[i].handler(args);
            return;
        }
    }

    UartDbg_Send(&uart_dbg, "unknown cmd: %s\r\n", line);
}

/* ===================== VOFA+ JustFloat 数据发送 ===================== */

void CmdHandler_SendVofa(void)
{
    if (!g_vofa_enable) return;

    float data[6];
    data[0] = angle_y;                              /* 当前偏航角 */
    data[1] = control_get_yaw();                    /* 目标偏航角 */
    data[2] = encoder_left.speed_cm_s;              /* 左轮速度 */
    data[3] = encoder_right.speed_cm_s;             /* 右轮速度 */
    data[4] = control_get_pwm_a();                  /* 左轮 PWM */
    data[5] = control_get_pwm_b();                  /* 右轮 PWM */

    UartDbg_SendFloat(&uart_dbg, data, 6);
}
