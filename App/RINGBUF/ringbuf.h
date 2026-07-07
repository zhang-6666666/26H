/* 通用环形缓冲区 — ISR 写 / 主循环读，单生产者单消费者，无锁 */
#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>

typedef struct {
    uint8_t *buffer;           /* 数据存储（外部提供静态数组） */
    uint32_t size;             /* 缓冲区大小，必须是 2 的幂 */
    volatile uint32_t head;    /* ISR 写入位置，只增不减，取模用 &(size-1) */
    volatile uint32_t tail;    /* 主循环读取位置 */
} ringbuf_t;

/* 初始化：传入静态数组和大小，size 自动对齐到 2 的幂 */
void ringbuf_init(ringbuf_t *rb, uint8_t *buf, uint32_t size);

/* ISR 侧：写入数据，返回实际写入字节数（缓冲区满时可能部分写入） */
uint32_t ringbuf_put(ringbuf_t *rb, const uint8_t *data, uint32_t len);

/* 主循环侧：读取数据，返回实际读取字节数（缓冲区空时返回 0） */
uint32_t ringbuf_get(ringbuf_t *rb, uint8_t *out, uint32_t len);

/* 可读字节数 */
uint32_t ringbuf_available(ringbuf_t *rb);

/* 剩余可写空间 */
uint32_t ringbuf_free(ringbuf_t *rb);

#endif
