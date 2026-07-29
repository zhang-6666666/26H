/* 串口命令解析 — 在线调 PID、设置目标速度/角度、VOFA+ 数据流控制 */
#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

#include <stdint.h>

/* 命令回调（从 uartdbg 收到一行时调用） */
void CmdHandler_Process(const char *line);

/* VOFA+ JustFloat 模式开关 */
extern volatile uint8_t g_vofa_enable;

/* 发送 VOFA+ 数据包（control_update 之后调用） */
void CmdHandler_SendVofa(void);

#endif
