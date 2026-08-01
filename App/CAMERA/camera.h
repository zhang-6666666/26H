/* K230 UART1 -> STM32 USART2 vision data receiver. */
#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>

#define CAMERA_TIMEOUT_MS  200U

typedef struct {
    int16_t pos_mm;                 /* 中心=0，左负右正 */
    int16_t vel_mm_s;
    int16_t target_mm;              /* K230 task target */
    uint16_t confidence_permille;   /* 0..1000 */
    uint16_t seq;
    uint32_t k230_timestamp_ms;
    uint32_t last_rx_ms;            /* STM32 local receive time */
    uint32_t frames_ok;
    uint32_t checksum_errors;
    uint32_t format_errors;
    uint32_t sequence_errors;
    uint8_t valid;                  /* K230 detection-valid flag */
    uint8_t received;               /* at least one verified frame */
    uint8_t fresh;                  /* main loop clears, ISR sets */
} CameraData;

extern volatile CameraData camera;

void Camera_Init(void);
void Camera_RxIsr(uint8_t byte);

/* Atomic copies from the ISR-owned global. */
uint8_t Camera_Take(CameraData *sample);  /* returns/clears fresh */
uint8_t Camera_Peek(CameraData *sample);  /* does not clear fresh */

/* True only for a verified valid frame not older than CAMERA_TIMEOUT_MS. */
uint8_t Camera_IsUsable(uint32_t now_ms);

#endif
