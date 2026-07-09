/* JY901P 传感器驱动 — DMA 环形接收 + IDLE 帧检测 + 环形缓冲 + SDK 胶水 */
#include "jy901p.h"
#include "wit_c_sdk.h"
#include "ringbuf.h"

/* ========================== 内部常量 ========================== */
#define DMA_RX_BUF_SIZE  64        /* DMA 硬件环形缓冲，必须是 2 的幂 */
#define RB_BUF_SIZE      128       /* 软件环形缓冲，必须是 2 的幂 */

/* ========================== 静态数据 ========================== */

/* 硬件句柄 */
static UART_HandleTypeDef *j_huart;        /* USART2 */
static DMA_HandleTypeDef  *j_hdma_rx;     /* DMA1_Channel6 */

/* DMA 硬件环形缓冲区（DMA 直接写入） */
static uint8_t  dma_buf[DMA_RX_BUF_SIZE];
static volatile uint32_t dma_last_ndtr;   /* 上一次 DMA NDTR 值，用于计算增量 */

/* 软件环形缓冲（ISR 写入，主循环读取） */
static ringbuf_t rb;
static uint8_t   rb_buf[RB_BUF_SIZE];

/* 角度数据 */
float  angle_r, angle_p, angle_y;        /* 角度 float，SDK 回调写入 */

/* 归零偏移 */
static float offset_r, offset_p, offset_y;


/* =================== SDK 回调（内部函数） =================== */

/* WitSerialWriteRegister 注册的回调：SDK 需要发送数据到传感器时调用（如写寄存器命令） */
static void sensor_send(uint8_t *data, uint32_t len)
{
    /* 暂停 DMA 接收 → 发送命令 → 恢复 DMA 接收，防止 IDLE 计数错乱 */
    HAL_UART_DMAStop(j_huart);
    HAL_UART_Transmit(j_huart, data, len, 10);
    HAL_UART_Receive_DMA(j_huart, dma_buf, DMA_RX_BUF_SIZE);
    dma_last_ndtr = DMA_RX_BUF_SIZE;
}

/* WitDelayMsRegister 注册的回调：SDK 需要延时等待时调用（如写寄存器后等待传感器处理） */
static void sensor_delay(uint16_t ms) { HAL_Delay(ms); }

/* WitRegisterCallBack 注册的回调：SDK 解析完一帧数据包后调用，通知上层有新数据到达 */
static void sensor_data_update(uint32_t uiReg, uint32_t uiRegNum)
{
    if (uiReg == Roll) {
        float r = (float)sReg[Roll]  * 180.0f / 32768.0f;
        float p = (float)sReg[Pitch] * 180.0f / 32768.0f;
        float y = (float)sReg[Yaw]   * 180.0f / 32768.0f;
        __disable_irq();
        angle_r = r - offset_r;
        angle_p = p - offset_p;
        angle_y = y - offset_y;
        __enable_irq();
    }
}

/* =================== 公开 API =================== */

void jy901p_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx)
{
    j_huart   = huart;
    j_hdma_rx = hdma_rx;

    /* 初始化环形缓冲 */
    ringbuf_init(&rb, rb_buf, RB_BUF_SIZE);

    /* 初始化 SDK：正常协议，地址 0x50 */
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(sensor_send);
    WitRegisterCallBack(sensor_data_update);
    WitDelayMsRegister(sensor_delay);

    /* 使能 UART IDLE 中断（HAL 默认不开启） */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    /* 启动 DMA 循环接收 */
    HAL_UART_Receive_DMA(huart, dma_buf, DMA_RX_BUF_SIZE);
    dma_last_ndtr = DMA_RX_BUF_SIZE;
}

void jy901p_uart_isr(UART_HandleTypeDef *huart)
{
    uint32_t mask = DMA_RX_BUF_SIZE - 1;

    /* 仅处理 USART2 */
    if (huart->Instance != USART2) return;

    /* 检测 IDLE：一帧数据收发完毕后 RX 线空闲 */
    if (!__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) return;

    /* 清除 IDLE 标志（读 SR 再读 DR） */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* 计算 DMA 新写入的字节数 */
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(j_hdma_rx);
    uint32_t head = DMA_RX_BUF_SIZE - ndtr;   /* DMA 写指针 */
    uint32_t last = DMA_RX_BUF_SIZE - dma_last_ndtr;
    uint32_t new_bytes = (head - last) & mask;

    if (new_bytes == 0) return;

    /* 逐字节写入软件环形缓冲（ISR 侧，put 是安全的） */
    for (uint32_t i = 0; i < new_bytes; i++) {
        uint8_t byte = dma_buf[(last + i) & mask];
        ringbuf_put(&rb, &byte, 1);
    }

    dma_last_ndtr = ndtr;
}

void jy901p_poll(void)
{
    /* 从环形缓冲取字节逐字节喂给 SDK 状态机 */
    uint8_t byte;
    while (ringbuf_available(&rb)) {
        ringbuf_get(&rb, &byte, 1);
        WitSerialDataIn(byte);
    }

    /* 将 SDK 解析完的数据包写入 sReg 并触发回调 */
    CopeWitData(ucRegIndex, usRegDataBuff, uiRegDataLen);
}

void jy901p_set_6axis(void)
{
    WitWriteReg(KEY, KEY_UNLOCK);
    HAL_Delay(1);
    WitWriteReg(AXIS6, ALGRITHM6);
    HAL_Delay(1);
    WitWriteReg(SAVE, SAVE_PARAM);
}

void jy901p_zero(void)
{
    float r, p, y;
    __disable_irq();
    r = angle_r + offset_r;
    p = angle_p + offset_p;
    y = angle_y + offset_y;
    __enable_irq();
    offset_r = r;
    offset_p = p;
    offset_y = y;
}
