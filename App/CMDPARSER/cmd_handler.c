/* 串口命令解析 + VOFA+ JustFloat 输出 */
#include "cmd_handler.h"
#include "control.h"
#include "encoder.h"
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
        "ping               check connection\r\n"
        "help               this list\r\n");
}


/* ===================== 命令表 ===================== */

typedef struct {
    const char *name;
    void (*handler)(const char *args);
} CmdEntry;

static const CmdEntry s_cmds[] = {
    {"ping",  cmd_ping},
    {"help",  cmd_help},
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

