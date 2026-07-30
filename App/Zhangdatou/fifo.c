#include "fifo.h"

/**********************************************************
*** Emm_V5.0 闭环步进驱动器
*** 编写作者：ZHANGDATOU
*** 硬件支持：张大头闭环步进驱动板
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN博客：https://blog.csdn.net/zhangdatou666
*** QQ交流群：262438510
**********************************************************/

__IO FIFO_t txFIFO = {0};               // 发送 FIFO 实例
__IO FIFO_t rxFIFO = {0};               // 接收 FIFO 实例

/**
  * @brief  初始化指定 FIFO（复位读写指针）
  * @param  fifo  要初始化的 FIFO 指针
  */
void fifo_init(volatile FIFO_t *fifo)
{
  fifo->ptrRead  = 0;
  fifo->ptrWrite = 0;
}

/**
  * @brief  入队：向 FIFO 尾部写入一个数据
  * @param  fifo  目标 FIFO 指针
  * @param  data  要存入的数据
  */
void fifo_enQueue(volatile FIFO_t *fifo, uint16_t data)
{
  fifo->buffer[fifo->ptrWrite] = data;

  ++fifo->ptrWrite;

  // 写指针到达缓冲区末尾时回绕到开头
  if (fifo->ptrWrite >= FIFO_SIZE)
  {
    fifo->ptrWrite = 0;
  }
}

/**
  * @brief  出队：从 FIFO 头部取出一个数据
  * @param  fifo  目标 FIFO 指针
  * @retval 取出的数据
  */
uint16_t fifo_deQueue(volatile FIFO_t *fifo)
{
  uint16_t element = 0;

  element = fifo->buffer[fifo->ptrRead];

  ++fifo->ptrRead;

  // 读指针到达缓冲区末尾时回绕到开头
  if (fifo->ptrRead >= FIFO_SIZE)
  {
    fifo->ptrRead = 0;
  }

  return element;
}

/**
  * @brief  判断 FIFO 是否为空
  * @param  fifo  目标 FIFO 指针
  * @retval true=空，false=非空
  */
bool fifo_isEmpty(volatile FIFO_t *fifo)
{
  if (fifo->ptrRead == fifo->ptrWrite)
  {
    return true;
  }

  return false;
}

/**
  * @brief  获取 FIFO 中当前缓存的数据个数
  * @param  fifo  目标 FIFO 指针
  * @retval 队列中的数据数量
  */
uint16_t fifo_queueLength(volatile FIFO_t *fifo)
{
  if (fifo->ptrRead <= fifo->ptrWrite)
  {
    // 写指针在后方，直接相减
    return (fifo->ptrWrite - fifo->ptrRead);
  }
  else
  {
    // 写指针已回绕，需要分段计算
    return (FIFO_SIZE - fifo->ptrRead + fifo->ptrWrite);
  }
}
