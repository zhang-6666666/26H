#include "motor_bujin.h"
#include "Emm_V5.h"
#include "fifo.h"
#include <stdbool.h>

/* ── 外部串口句柄 ─────────────────────────────────────── */
extern UART_HandleTypeDef huart3;

/* ── 电机状态全局变量 ─────────────────────────────────── */
volatile Motor_State_t motor_state = {0};

/* ── 接收数据包解析状态机 ─────────────────────────────── */
#define RX_WAIT_ADDR     0     // 等待地址字节
#define RX_WAIT_LEN       1     // 等待长度字节
#define RX_WAIT_DATA      2     // 等待数据 + 校验字节

static uint8_t  rx_state = RX_WAIT_ADDR;
static uint8_t  rx_buf[32];      // 接收缓冲区
static uint8_t  rx_idx = 0;      // 当前接收位置
static uint8_t  rx_len = 0;      // 期望数据长度

/* ── 通信层 ──────────────────────────────────────────── */

/**
  * @brief  初始化电机通信（发送 + 接收 FIFO）
  */
void Motor_Comm_Init(void)
{
    fifo_init(&txFIFO);
    fifo_init(&rxFIFO);
    rx_state = RX_WAIT_ADDR;

    /* 使能 RXNE 中断，开始接收驱动板数据 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

/**
  * @brief  被 Emm_V5.c 调用的底层发送函数
  * @note   将指令压入发送 FIFO，并触发 TXE 中断
  */
void usart_SendCmd(uint8_t *cmd, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        fifo_enQueue(&txFIFO, (uint16_t)cmd[i]);
    }
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);
}

/**
  * @brief  发送处理：从 TX_FIFO 取字节写入串口
  * @note   放在 UART TXE 中断中调用
  */
void Motor_Tx_Process(void)
{
    if (!fifo_isEmpty(&txFIFO)) {
        uint8_t byte_to_send = (uint8_t)fifo_deQueue(&txFIFO);
        USART3->DR = byte_to_send;
    } else {
        __HAL_UART_DISABLE_IT(&huart3, UART_IT_TXE);
    }
}

/**
  * @brief  接收处理：将收到的字节压入接收 FIFO
  * @note   放在 UART RXNE 中断中调用
  * @param  byte  收到的单个字节
  */
void Motor_Rx_Handler(uint8_t byte)
{
    fifo_enQueue(&rxFIFO, (uint16_t)byte);
}

/**
  * @brief  接收解析：从 RX_FIFO 取数据，按协议解析完整数据包
  * @note   放在主循环中轮询
  *
  * 协议格式：地址(1B) + 长度(1B) + 数据(NB) + 校验(1B=0x6B)
  * 解析后更新 motor_state 全局变量
  */
void Motor_Rx_Process(void)
{
    while (!fifo_isEmpty(&rxFIFO)) {
        uint8_t byte = (uint8_t)fifo_deQueue(&rxFIFO);

        switch (rx_state) {

        case RX_WAIT_ADDR:
            // 地址字节：只接受本机地址或广播地址
            if (byte == STEP_MOTOR_ADDR || byte == 0xFF) {
                rx_buf[0] = byte;
                rx_idx = 1;
                rx_state = RX_WAIT_LEN;
            }
            // 非本机地址则丢弃，继续等待
            break;

        case RX_WAIT_LEN:
            rx_len = byte;                    // 数据长度（含校验）
            rx_buf[rx_idx++] = byte;
            if (rx_len > 0 && rx_len < 32) {
                rx_state = RX_WAIT_DATA;
            } else {
                // 长度异常，重置
                rx_state = RX_WAIT_ADDR;
            }
            break;

        case RX_WAIT_DATA:
            rx_buf[rx_idx++] = byte;
            if (rx_idx >= rx_len + 2) {       // 地址(1) + 长度(1) + 数据(rx_len)
                // 校验：最后一个字节应为 0x6B
                if (rx_buf[rx_idx - 1] == 0x6B) {
                    // 解析数据包，更新电机状态
                    uint8_t  data_len = rx_len - 1;   // 去掉校验字节
                    uint8_t *data     = &rx_buf[2];   // 跳过地址和长度

                    // 根据命令码判断响应类型并解析
                    // 命令码在发送指令的第3字节(rx_buf[2])，数据从 rx_buf[3] 开始
                    switch (rx_buf[2]) {
                    case 0x35:  // 读取实时转速响应：data[0]高8位, data[1]低8位
                        if (data_len >= 2) {
                            motor_state.cur_speed = (int16_t)((data[0] << 8) | data[1]);
                        }
                        break;
                    case 0x36:  // 读取实时位置响应：data[0]~data[3]
                        if (data_len >= 4) {
                            motor_state.cur_pos = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
                        }
                        break;
                    case 0x3A:  // 状态标志响应
                        if (data_len >= 1) {
                            motor_state.is_enabled = (data[0] & 0x01) ? 1 : 0;
                            motor_state.is_moving  = (data[0] & 0x02) ? 1 : 0;
                            motor_state.has_error  = (data[0] & 0x04) ? 1 : 0;
                        }
                        break;
                    default:
                        // 其他应答暂不解析
                        break;
                    }
                }
                // 无论校验是否通过，都重置状态机准备下一包
                rx_state = RX_WAIT_ADDR;
            }
            break;
        }
    }
}

/* ── 步进电机简易控制接口 ─────────────────────────────── */

/**
  * @brief  步进电机上电初始化
  *         1. 初始化通信 FIFO
  *         2. 设置为闭环模式（不存储，掉电恢复默认）
  *         3. 使能电机
  */
void Motor_Step_Init(void)
{
    Motor_Comm_Init();

    // 设置为闭环模式 (ctrl_mode=2)，不存储
    Emm_V5_Modify_Ctrl_Mode(STEP_MOTOR_ADDR, false, 2);

    // 使能电机
    Emm_V5_En_Control(STEP_MOTOR_ADDR, true, false);
}

/**
  * @brief  速度模式控制
  * @param  rpm  目标转速：正=CW，负=CCW，0=停止
  */
void Motor_Step_SetSpeed(int16_t rpm)
{
    if (rpm == 0) {
        Motor_Step_Stop();
        return;
    }

    uint8_t  dir = (rpm > 0) ? 0 : 1;         // 0=CW, 非0=CCW
    uint16_t vel = (uint16_t)(rpm > 0 ? rpm : -rpm);
    uint8_t  acc = 10;                         // 默认加速度

    Emm_V5_Vel_Control(STEP_MOTOR_ADDR, dir, vel, acc, false);
}

/**
  * @brief  相对位置移动
  * @param  pulse  脉冲数，正=CW / 负=CCW（1圈=PPR脉冲，可用 REV2P / DEG2P）
  * @param  rpm    运动速度
  */
void Motor_Step_MoveRel(int32_t pulse, uint16_t rpm)
{
    uint8_t  dir = (pulse >= 0) ? 0 : 1;
    uint32_t clk = (uint32_t)(pulse >= 0 ? pulse : -pulse);
    uint8_t  acc = 10;

    Emm_V5_Pos_Control(STEP_MOTOR_ADDR, dir, rpm, acc, clk, false, false);
}

/**
  * @brief  绝对位置移动
  * @param  pulse  目标位置脉冲数（1圈=PPR脉冲，可用 REV2P / DEG2P）
  * @param  rpm    运动速度
  */
void Motor_Step_MoveTo(int32_t pulse, uint16_t rpm)
{
    uint8_t  dir = (pulse >= motor_state.cur_pos) ? 0 : 1;
    uint32_t clk = (uint32_t)(pulse >= motor_state.cur_pos ? (pulse - motor_state.cur_pos) : (motor_state.cur_pos - pulse));
    uint8_t  acc = 10;

    Emm_V5_Pos_Control(STEP_MOTOR_ADDR, dir, rpm, acc, clk, true, false);
}

/**
  * @brief  减速停止（保持使能）
  */
void Motor_Step_Stop(void)
{
    Emm_V5_Stop_Now(STEP_MOTOR_ADDR, false);
}

/**
  * @brief  紧急停止（直接失能电机，惯性滑行）
  */
void Motor_Step_EmergencyStop(void)
{
    Emm_V5_En_Control(STEP_MOTOR_ADDR, false, false);
}
