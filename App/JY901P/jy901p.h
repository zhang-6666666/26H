/* JY901P 传感器驱动 — DMA 环形接收 + IDLE 帧检测 + SDK 胶水 */
#ifndef JY901P_H
#define JY901P_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 初始化：必须在 MX_USART2_UART_Init 之后调用
   huart: USART2 句柄
   hdma_rx: USART2_RX 的 DMA 句柄（CubeMX 生成的 hdma_usart2_rx） */
void jy901p_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx);

/* USART2 中断处理：在 stm32f1xx_it.c 的 USART2_IRQHandler 末尾调用 */
void jy901p_uart_isr(UART_HandleTypeDef *huart);

/* 主循环轮询：从环形缓冲取字节喂 SDK，检测是否有新角度 */
void jy901p_poll(void);

/* 角度数据（SDK 回调直接写入，外部只读） */
extern float angle_r, angle_p, angle_y;

#endif
