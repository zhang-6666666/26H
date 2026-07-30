#ifndef __FIFO_H
#define __FIFO_H

#include <stdint.h>
#include <stdbool.h>

#define __IO  volatile

/**********************************************************
*** Emm_V5.0 闭环步进驱动器
*** 编写作者：ZHANGDATOU
*** 硬件支持：张大头闭环步进驱动板
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN博客：https://blog.csdn.net/zhangdatou666
*** QQ交流群：262438510
**********************************************************/

#define   FIFO_SIZE   128               // FIFO缓冲区大小

/* FIFO队列结构体 */
typedef struct {
  uint16_t buffer[FIFO_SIZE];           // 数据缓冲区
  __IO uint8_t ptrWrite;                // 写指针
  __IO uint8_t ptrRead;                 // 读指针
} FIFO_t;

extern __IO FIFO_t txFIFO;              // 发送 FIFO 实例
extern __IO FIFO_t rxFIFO;              // 接收 FIFO 实例

void fifo_init(volatile FIFO_t *fifo);                              // 初始化指定 FIFO
void fifo_enQueue(volatile FIFO_t *fifo, uint16_t data);             // 入队
uint16_t fifo_deQueue(volatile FIFO_t *fifo);                        // 出队
bool fifo_isEmpty(volatile FIFO_t *fifo);                            // 判断队列是否为空
uint16_t fifo_queueLength(volatile FIFO_t *fifo);                    // 获取队列中数据个数

#endif
