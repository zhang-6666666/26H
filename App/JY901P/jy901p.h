/* JY901P 传感器驱动 — DMA 环形接收 + IDLE 帧检测 + SDK 胶水 */
#ifndef JY901P_H
#define JY901P_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 初始化：必须在 MX_USART2_UART_Init 之后调用
   huart: USART2 句柄
   hdma_rx: USART2_RX 的 DMA 句柄（CubeMX 生成的 hdma_usart2_rx） */
void jy901p_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx);

/* 一次性传感器配置（AXIS6、RRATE），保存到 Flash */
void jy901p_config(void);

/* 启动陀螺仪零偏校准（传感器需静止水平），主循环需持续 jy901p_poll() 约 2-3s */
void jy901p_calibrate_gyro(void);

/* USART2 中断处理：在 stm32f1xx_it.c 的 USART2_IRQHandler 末尾调用 */
void jy901p_uart_isr(UART_HandleTypeDef *huart);

/* 主循环轮询：从环形缓冲取字节喂 SDK，检测是否有新角度 */
void jy901p_poll(void);

/* 将当前角度归零（记录当前角度为参考点，此后 angle_* 输出相对角度） */
void jy901p_zero(void);

/* 角度数据（SDK 回调直接写入，外部只读） */
extern float angle_r, angle_p, angle_y;

#endif
