/* 通用环形缓冲区 — ISR 写 / 主循环读，单生产者单消费者，无锁 */
#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>

typedef struct {
    uint8_t *buffer;
    uint32_t size;
    volatile uint32_t head;
    volatile uint32_t tail;
} RingBuf;

void RingBuf_Init(RingBuf *rb, uint8_t *buf, uint32_t size);
uint32_t RingBuf_Put(RingBuf *rb, const uint8_t *data, uint32_t len);
uint32_t RingBuf_Get(RingBuf *rb, uint8_t *out, uint32_t len);
uint32_t RingBuf_Avail(RingBuf *rb);
uint32_t RingBuf_Free(RingBuf *rb);

#endif
