/* 串口调试模块 — DMA 发送 + 接收行缓冲 + 命令回调 + JustFloat */
#ifndef UARTDBG_H
#define UARTDBG_H

#include "stm32f1xx_hal.h"
#include "ringbuf.h"
#include <stdint.h>

#define UARTDBG_TX_BUF_SIZE   128
#define UARTDBG_DMA_RX_SIZE   64
#define UARTDBG_RB_BUF_SIZE   128
#define UARTDBG_CMD_BUF_SIZE  64

/* 命令回调：收到完整一行时调用，line 以 \0 结尾（不含 \r\n） */
typedef void (*UartDbg_CmdCb)(const char *line);

typedef struct {
    UART_HandleTypeDef  *huart;
    DMA_HandleTypeDef   *hdma_tx;
    volatile uint8_t     tx_busy;
    char                 tx_buf[UARTDBG_TX_BUF_SIZE];

    /* RX — DMA 循环接收 */
    DMA_HandleTypeDef   *hdma_rx;
    uint8_t              dma_rx_buf[UARTDBG_DMA_RX_SIZE];
    volatile uint32_t    dma_rx_last_ndtr;
    RingBuf              rx_rb;
    uint8_t              rx_rb_buf[UARTDBG_RB_BUF_SIZE];

    /* 行缓冲 */
    char                 cmd_buf[UARTDBG_CMD_BUF_SIZE];
    uint32_t             cmd_len;
    UartDbg_CmdCb        cmd_cb;
} UartDbg;

void UartDbg_Init(UartDbg *dbg, UART_HandleTypeDef *huart,
                  DMA_HandleTypeDef *hdma_tx, DMA_HandleTypeDef *hdma_rx);
void UartDbg_Send(UartDbg *dbg, const char *fmt, ...);

/* VOFA+ JustFloat：发送 count 个 float，尾部自动加尾帧 0x00 0x00 0x80 0x7F */
void UartDbg_SendFloat(UartDbg *dbg, const float *data, uint32_t count);

void UartDbg_SetCmdCb(UartDbg *dbg, UartDbg_CmdCb cb);
void UartDbg_TxCpltCb(UartDbg *dbg);
void UartDbg_RxIdleHandler(UartDbg *dbg);

/* 从 ringbuf 取字节拼行，遇到 \n/\r 调用回调 */
void UartDbg_Poll(UartDbg *dbg);

extern UartDbg uart_dbg;

#endif
