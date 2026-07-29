/* 串口调试模块 — DMA TX 发送 + DMA RX IDLE 帧检测 + 行缓冲 + JustFloat */
#include "uartdbg.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

UartDbg uart_dbg;

void UartDbg_Init(UartDbg *dbg, UART_HandleTypeDef *huart,
                  DMA_HandleTypeDef *hdma_tx, DMA_HandleTypeDef *hdma_rx)
{
    dbg->huart   = huart;
    dbg->hdma_tx = hdma_tx;
    dbg->hdma_rx = hdma_rx;
    dbg->tx_busy = 0;
    dbg->cmd_len = 0;
    dbg->cmd_cb  = NULL;

    if (hdma_rx) {
        RingBuf_Init(&dbg->rx_rb, dbg->rx_rb_buf, UARTDBG_RB_BUF_SIZE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
        HAL_UART_Receive_DMA(huart, dbg->dma_rx_buf, UARTDBG_DMA_RX_SIZE);
        dbg->dma_rx_last_ndtr = UARTDBG_DMA_RX_SIZE;
    }
}

void UartDbg_Send(UartDbg *dbg, const char *fmt, ...)
{
    if (dbg->tx_busy) return;

    va_list args;
    va_start(args, fmt);
    vsnprintf(dbg->tx_buf, UARTDBG_TX_BUF_SIZE, fmt, args);
    va_end(args);

    dbg->tx_busy = 1;
    HAL_UART_Transmit_DMA(dbg->huart, (uint8_t *)dbg->tx_buf, strlen(dbg->tx_buf));
}

void UartDbg_SendFloat(UartDbg *dbg, const float *data, uint32_t count)
{
    /* JustFloat: count*4 字节 float 数据 + 4 字节尾帧 0x00 0x00 0x80 0x7F */
    uint32_t total = count * 4 + 4;
    if (total > UARTDBG_TX_BUF_SIZE) return;
    if (dbg->tx_busy) return;

    uint8_t *p = (uint8_t *)dbg->tx_buf;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t v;
        memcpy(&v, &data[i], 4);
        *p++ = v & 0xFF;
        *p++ = (v >> 8) & 0xFF;
        *p++ = (v >> 16) & 0xFF;
        *p++ = (v >> 24) & 0xFF;
    }
    /* 尾帧 */
    *p++ = 0x00; *p++ = 0x00; *p++ = 0x80; *p++ = 0x7F;

    dbg->tx_busy = 1;
    HAL_UART_Transmit_DMA(dbg->huart, (uint8_t *)dbg->tx_buf, total);
}

void UartDbg_SetCmdCb(UartDbg *dbg, UartDbg_CmdCb cb)
{
    dbg->cmd_cb = cb;
}

void UartDbg_TxCpltCb(UartDbg *dbg)
{
    dbg->tx_busy = 0;
}

void UartDbg_RxIdleHandler(UartDbg *dbg)
{
    if (!dbg->hdma_rx) return;

    uint32_t mask = UARTDBG_DMA_RX_SIZE - 1;
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(dbg->hdma_rx);
    uint32_t head = UARTDBG_DMA_RX_SIZE - ndtr;
    uint32_t last = UARTDBG_DMA_RX_SIZE - dbg->dma_rx_last_ndtr;
    uint32_t new_bytes = (head - last) & mask;

    for (uint32_t i = 0; i < new_bytes; i++) {
        uint8_t byte = dbg->dma_rx_buf[(last + i) & mask];
        RingBuf_Put(&dbg->rx_rb, &byte, 1);
    }

    dbg->dma_rx_last_ndtr = ndtr;
}

void UartDbg_Poll(UartDbg *dbg)
{
    if (!dbg->hdma_rx || !dbg->cmd_cb) return;

    uint8_t byte;
    while (RingBuf_Avail(&dbg->rx_rb)) {
        RingBuf_Get(&dbg->rx_rb, &byte, 1);

        if (byte == '\r' || byte == '\n') {
            if (dbg->cmd_len > 0) {
                dbg->cmd_buf[dbg->cmd_len] = '\0';
                dbg->cmd_cb(dbg->cmd_buf);
                dbg->cmd_len = 0;
            }
        } else {
            if (dbg->cmd_len < UARTDBG_CMD_BUF_SIZE - 1) {
                dbg->cmd_buf[dbg->cmd_len++] = (char)byte;
            }
        }
    }
}

/* HAL 弱回调：USART DMA 发送完成 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uart_dbg.tx_busy = 0;
    }
}
