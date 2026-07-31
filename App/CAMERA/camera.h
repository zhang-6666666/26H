/* K230 摄像头数据接收 — USART2 RXNE 中断 */
#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>

typedef struct {
    int16_t  pos_mm;       /* 最新位置 (mm)，中心=0 */
    uint8_t  fresh;        /* 主循环清 0，ISR 置 1 */
} CameraData;

extern volatile CameraData camera;

void Camera_Init(void);    /* 使能 USART2 RXNE 中断 */
void Camera_RxIsr(uint8_t byte);   /* ISR 中调用 */

#endif
