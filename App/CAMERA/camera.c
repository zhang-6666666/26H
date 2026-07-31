/* K230 UART1 ASCII frames received by STM32 USART2 RXNE interrupt. */
#include "camera.h"
#include "camera_protocol.h"
#include "usart.h"

volatile CameraData camera;
static CameraProtocolParser s_parser;

static void camera_copy(CameraData *sample)
{
    sample->pos_mm = camera.pos_mm;
    sample->vel_mm_s = camera.vel_mm_s;
    sample->confidence_permille = camera.confidence_permille;
    sample->seq = camera.seq;
    sample->k230_timestamp_ms = camera.k230_timestamp_ms;
    sample->last_rx_ms = camera.last_rx_ms;
    sample->frames_ok = camera.frames_ok;
    sample->checksum_errors = camera.checksum_errors;
    sample->format_errors = camera.format_errors;
    sample->sequence_errors = camera.sequence_errors;
    sample->valid = camera.valid;
    sample->received = camera.received;
    sample->fresh = camera.fresh;
}

void Camera_Init(void)
{
    camera.pos_mm = 0;
    camera.vel_mm_s = 0;
    camera.confidence_permille = 0U;
    camera.seq = 0U;
    camera.k230_timestamp_ms = 0U;
    camera.last_rx_ms = 0U;
    camera.frames_ok = 0U;
    camera.checksum_errors = 0U;
    camera.format_errors = 0U;
    camera.sequence_errors = 0U;
    camera.valid = 0U;
    camera.received = 0U;
    camera.fresh = 0U;
    CameraProtocol_Init(&s_parser);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void Camera_RxIsr(uint8_t byte)
{
    CameraProtocolFrame frame;
    CameraParseResult result = CameraProtocol_Push(&s_parser, byte, &frame);

    if (result == CAMERA_PARSE_CHECKSUM_ERROR) {
        camera.checksum_errors++;
        return;
    }
    if (result == CAMERA_PARSE_FORMAT_ERROR) {
        camera.format_errors++;
        return;
    }
    if (result != CAMERA_PARSE_FRAME) {
        return;
    }

    if (camera.received) {
        uint16_t expected = (uint16_t)(camera.seq + 1U);
        if (frame.seq != expected) {
            camera.sequence_errors++;
        }
    }

    camera.seq = frame.seq;
    camera.k230_timestamp_ms = frame.timestamp_ms;
    camera.valid = frame.valid;
    camera.pos_mm = frame.valid ? frame.pos_mm : 0;
    camera.vel_mm_s = frame.valid ? frame.vel_mm_s : 0;
    camera.confidence_permille = frame.valid ? frame.confidence_permille : 0U;
    camera.last_rx_ms = HAL_GetTick();
    camera.frames_ok++;
    camera.received = 1U;
    camera.fresh = 1U;  /* Publish after every other field is complete. */
}

uint8_t Camera_Take(CameraData *sample)
{
    uint32_t primask;
    uint8_t fresh;

    if (sample == 0) {
        return 0U;
    }
    if (!camera.fresh) {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    fresh = camera.fresh;
    camera_copy(sample);
    camera.fresh = 0U;
    __set_PRIMASK(primask);
    return fresh;
}

uint8_t Camera_Peek(CameraData *sample)
{
    uint32_t primask;

    if (sample == 0) {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    camera_copy(sample);
    __set_PRIMASK(primask);
    return sample->received;
}

uint8_t Camera_IsUsable(uint32_t now_ms)
{
    uint32_t age_ms;
    if (!camera.received || !camera.valid) {
        return 0U;
    }
    age_ms = now_ms - camera.last_rx_ms;
    return (age_ms <= CAMERA_TIMEOUT_MS) ? 1U : 0U;
}
