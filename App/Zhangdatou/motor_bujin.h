#ifndef __MOTOR_BUJIN_H
#define __MOTOR_BUJIN_H

#include "main.h"

/* ── 步进电机参数 ────────────────────────────────────────── */
#define PPR                   51200                       // 每圈脉冲数 (128细分 × 400步/圈)
#define DEG2P(a)              ((a) * PPR / 360)           // 角度→脉冲
#define REV2P(r)              ((r) * PPR)                 // 圈数→脉冲

/* ── 步进电机驱动地址（单驱动器默认 0） ────────────────── */
#define STEP_MOTOR_ADDR       0

/* ── 电机状态结构体（由接收解析自动更新） ──────────────── */
typedef struct {
    int32_t  cur_pos;           // 当前位置（编码器脉冲数）
    int16_t  cur_speed;         // 当前转速 (RPM)
    uint8_t  is_enabled;        // 使能状态：0=失能，1=使能
    uint8_t  is_moving;         // 运动状态：0=停止，1=运动中
    uint8_t  has_error;         // 故障标志：0=正常，1=有故障
} Motor_State_t;

/* ── 全局电机状态（只读，由接收中断更新） ──────────────── */
extern volatile Motor_State_t motor_state;

/* ── 底层发送函数（由 Emm_V5.c 调用） ──────────────────── */
extern void usart_SendCmd(uint8_t *cmd, uint8_t len);   // 发送指令包（压入 TX_FIFO 并触发 TXE 中断）

/* ── 通信层接口（中断 / 主循环调用） ───────────────────── */
void Motor_Comm_Init(void);                     // 初始化通信（FIFO + UART）
void Motor_Tx_Process(void);                    // 发送处理：从 TX_FIFO 取字节→串口发送（放 TXE 中断或主循环）
void Motor_Rx_Handler(uint8_t byte);            // 接收处理：将收到的字节压入 RX_FIFO（放 RXNE 中断）
void Motor_Rx_Process(void);                    // 接收解析：从 RX_FIFO 解析完整数据包，更新 motor_state（放主循环）

/* ── 步进电机简易控制接口 ─────────────────────────────── */
void Motor_Step_Init(void);                     // 上电初始化：通信→闭环模式→使能
void Motor_Step_SetSpeed(int16_t rpm);          // 速度模式：正=CW，负=CCW，0=停止
void Motor_Step_MoveRel(int32_t pulse, uint16_t rpm);   // 相对位置移动（pulse 为脉冲数，用 REV2P/DEG2P 转换）
void Motor_Step_MoveTo(int32_t pulse, uint16_t rpm);    // 绝对位置移动（pulse 为脉冲数，用 REV2P/DEG2P 转换）
void Motor_Step_Stop(void);                     // 减速停止
void Motor_Step_EmergencyStop(void);             // 紧急停止（直接失能电机）

#endif
