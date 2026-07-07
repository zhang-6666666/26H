/* 通用环形缓冲区实现 */
#include "ringbuf.h"

/* 将 size 向上对齐到最近的 2 的幂，用 &(size-1) 代替 % 取模 */
static uint32_t roundup_pow2(uint32_t n)
{
    n--;
    n |= n >> 1;  n |= n >> 2;  n |= n >> 4;
    n |= n >> 8;  n |= n >> 16;
    return n + 1;
}

void ringbuf_init(ringbuf_t *rb, uint8_t *buf, uint32_t size)
{
    rb->buffer = buf;
    rb->size   = roundup_pow2(size);
    rb->head   = 0;
    rb->tail   = 0;
}

uint32_t ringbuf_put(ringbuf_t *rb, const uint8_t *data, uint32_t len)
{
    uint32_t mask = rb->size - 1;
    uint32_t free = rb->size - (rb->head - rb->tail);

    if (len > free) len = free;          /* 空间不够，只写能写下的部分 */

    for (uint32_t i = 0; i < len; i++) {
        rb->buffer[(rb->head + i) & mask] = data[i];
    }
    rb->head += len;                      /* head 只增不减，溢出由 uint32_t 回绕 */
    return len;
}

uint32_t ringbuf_get(ringbuf_t *rb, uint8_t *out, uint32_t len)
{
    uint32_t mask = rb->size - 1;
    uint32_t avail = rb->head - rb->tail;

    if (len > avail) len = avail;         /* 数据不够，只读能读的部分 */

    for (uint32_t i = 0; i < len; i++) {
        out[i] = rb->buffer[(rb->tail + i) & mask];
    }
    rb->tail += len;
    return len;
}

uint32_t ringbuf_available(ringbuf_t *rb)
{
    return rb->head - rb->tail;
}

uint32_t ringbuf_free(ringbuf_t *rb)
{
    return rb->size - (rb->head - rb->tail);
}
