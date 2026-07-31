/* K230 摄像头数据接收 — USART2 RXNE 中断，2 字节 int16 */
#include "camera.h"
#include "usart.h"

volatile CameraData camera;

void Camera_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void Camera_RxIsr(uint8_t byte)
{
    static uint8_t  idx;
    static uint16_t buf;

    ((uint8_t *)&buf)[idx & 1] = byte;
    idx++;

    if (idx >= 2) {
        camera.pos_mm = (int16_t)buf;
        camera.fresh  = 1;
        idx = 0;
    }
}
