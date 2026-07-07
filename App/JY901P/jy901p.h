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

/* 返回非 0 表示有新角度数据就绪 */
uint8_t jy901p_angle_ready(void);

/* 取角度并直接返回格式化字符串（×10 整数，避免 printf 浮点）
   例: "Roll=-1.8 Pitch=+0.7 Yaw=132.9" */
const char *jy901p_angle_str(void);

/* 读取角度（×10 整数，避免 printf 浮点） */
void jy901p_read_angle(int32_t *roll_10, int32_t *pitch_10, int32_t *yaw_10);

#endif
