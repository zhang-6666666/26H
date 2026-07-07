/* JY901P 传感器驱动 — DMA 环形接收 + IDLE 帧检测 + 环形缓冲 + SDK 胶水 */
#include "jy901p.h"
#include "wit_c_sdk.h"
#include "ringbuf.h"
#include <stdio.h>

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
static volatile uint8_t  has_angle;       /* 非 0 = 收到新角度 */
static int32_t angle_r10, angle_p10, angle_y10;  /* 角度 ×10 */

/* =================== SDK 回调（内部函数） =================== */

static void sensor_send(uint8_t *data, uint32_t len)
{
    /* 暂停 DMA 接收 → 发送命令 → 恢复 DMA 接收，防止 IDLE 计数错乱 */
    HAL_UART_DMAStop(j_huart);
    HAL_UART_Transmit(j_huart, data, len, 10);
    HAL_UART_Receive_DMA(j_huart, dma_buf, DMA_RX_BUF_SIZE);
    dma_last_ndtr = DMA_RX_BUF_SIZE;
}

static void sensor_delay(uint16_t ms) { HAL_Delay(ms); }

static void sensor_data_update(uint32_t uiReg, uint32_t uiRegNum)
{
    if (uiReg == Roll) {
        /* 直接从 sReg 取数据并计算，不等主循环 */
        int32_t r10 = (int32_t)sReg[Roll]  * 1800 / 32768;
        int32_t p10 = (int32_t)sReg[Pitch] * 1800 / 32768;
        int32_t y10 = (int32_t)sReg[Yaw]   * 1800 / 32768;
        /* 关中断保护 32 位写入的原子性 */
        __disable_irq();
        angle_r10 = r10;  angle_p10 = p10;  angle_y10 = y10;
        has_angle = 1;
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

uint8_t jy901p_angle_ready(void)
{
    return has_angle;
}

/* 取角度并格式化为字符串（×10 整数，避免 printf 浮点），例: "Roll=-1.8 Pitch=+0.7 Yaw=132.9" */
const char *jy901p_angle_str(void)
{
    static char buf[64];
    int32_t r, p, y;
    __disable_irq();
    r = angle_r10;  p = angle_p10;  y = angle_y10;
    has_angle = 0;
    __enable_irq();

    int rs = r < 0, ra = rs ? -r : r;
    int ps = p < 0, pa = ps ? -p : p;
    int ys = y < 0, ya = ys ? -y : y;

    sprintf(buf, "Roll=%c%d.%d Pitch=%c%d.%d Yaw=%c%d.%d",
            rs ? '-' : '+', ra / 10, ra % 10,
            ps ? '-' : '+', pa / 10, pa % 10,
            ys ? '-' : '+', ya / 10, ya % 10);
    return buf;
}

void jy901p_read_angle(int32_t *roll_10, int32_t *pitch_10, int32_t *yaw_10)
{
    __disable_irq();
    *roll_10  = angle_r10;
    *pitch_10 = angle_p10;
    *yaw_10   = angle_y10;
    has_angle = 0;
    __enable_irq();
}
