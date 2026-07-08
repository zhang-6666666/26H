/* WIT 维特智能传感器 SDK — 公开 API 头文件
 * 支持协议：串口正常协议 / Modbus / CAN / I2C
 * 官网：http://wit-motion.cn/
 */

#ifndef __WIT_C_SDK_H
#define __WIT_C_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "REG.h"

/* -------------------- SDK 状态码 -------------------- */
#define WIT_HAL_OK      (0)     /* 成功 */
#define WIT_HAL_BUSY    (-1)    /* 忙 */
#define WIT_HAL_TIMEOUT (-2)    /* 超时 */
#define WIT_HAL_ERROR   (-3)    /* 通用错误 */
#define WIT_HAL_NOMEM   (-4)    /* 内存不足 */
#define WIT_HAL_EMPTY   (-5)    /* 资源为空（如回调未注册） */
#define WIT_HAL_INVAL   (-6)    /* 参数无效 */

/* SDK 内部接收缓冲区大小（字节） */
#define WIT_DATA_BUFF_SIZE  256

/* 通信协议类型 */
#define WIT_PROTOCOL_NORMAL 0   /* 串口正常协议（0x55 帧头） */
#define WIT_PROTOCOL_MODBUS 1   /* Modbus 协议 */
#define WIT_PROTOCOL_CAN    2   /* CAN 协议 */
#define WIT_PROTOCOL_I2C    3   /* I2C 协议 */


/* ==================== 串口（UART）相关 ==================== */

/* 解析 SDK 内部拼好的数据包：将 usRegDataBuff 里的原始值写入 sReg[] 并触发回调 */
void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen);

/* 串口发送回调函数类型：SDK 需要向外发送数据时调用 */
typedef void (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);

/* 注册串口发送回调（如 HAL_UART_Transmit） */
int32_t WitSerialWriteRegister(SerialWrite write_func);

/* 喂一个字节给 SDK 状态机：SDK 内部自动拼包、校验、解析 */
void WitSerialDataIn(uint8_t ucData);


/* ==================== I2C 相关 ==================== */

/* I2C 写函数类型 */
typedef int32_t (*WitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

/* I2C 读函数类型 */
typedef int32_t (*WitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

/* 注册 I2C 读写回调 */
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func);


/* ==================== CAN 相关 ==================== */

/* CAN 发送回调函数类型 */
typedef void (*CanWrite)(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);

/* 注册 CAN 发送回调 */
int32_t WitCanWriteRegister(CanWrite write_func);


/* ==================== 延时回调 ==================== */

/* 毫秒延时回调函数类型：SDK 写寄存器后需等待传感器处理 */
typedef void (*DelaymsCb)(uint16_t ucMs);

/* 注册毫秒延时回调（如 HAL_Delay） */
int32_t WitDelayMsRegister(DelaymsCb delayms_func);


/* ==================== CAN 数据输入 ==================== */

/* 喂一帧 CAN 数据（8 字节）给 SDK 解析 */
void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen);


/* ==================== 数据更新回调 & 寄存器读写 ==================== */

/* 数据更新回调函数类型：SDK 解析完一帧数据后调用，通知上层哪个寄存器更新了多少个值 */
typedef void (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);

/* 注册数据更新回调 */
int32_t WitRegisterCallBack(RegUpdateCb update_func);

/* 向传感器写一个寄存器（SDK 内部通过已注册的 SerialWrite/CAN/I2C 发送） */
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData);

/* 向传感器读若干寄存器（从 uiReg 开始，读 uiReadNum 个） */
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);

/* SDK 初始化：设置协议类型和设备地址 */
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);

/* SDK 反初始化：清空所有回调指针和状态 */
void WitDeInit(void);


/* ==================== 传感器配置函数 ==================== */

int32_t WitStartAccCali(void);       /* 开始加速度校准（需水平放置） */
int32_t WitStopAccCali(void);        /* 停止加速度校准并保存 */
int32_t WitStartMagCali(void);       /* 开始磁场校准 */
int32_t WitStopMagCali(void);        /* 停止磁场校准 */
int32_t WitSetUartBaud(int32_t uiBaudIndex);   /* 设置串口波特率 */
int32_t WitSetBandwidth(int32_t uiBaudWidth);  /* 设置低通滤波带宽 */
int32_t WitSetOutputRate(int32_t uiRate);      /* 设置数据输出频率 */
int32_t WitSetContent(int32_t uiRsw);          /* 设置自动输出内容（如只输出角度 RSW_ANGLE） */
int32_t WitSetCanBaud(int32_t uiBaudIndex);    /* 设置 CAN 波特率 */

/* 范围检查工具：sTemp 是否在 [sMin, sMax] 之间 */
char CheckRange(short sTemp, short sMin, short sMax);


/* ==================== SDK 全局变量（外部可直接读取） ==================== */

extern int16_t sReg[REGSIZE];        /* 寄存器镜像数组：sReg[Roll]=横滚角原始值，sReg[Pitch]=俯仰角原始值 … */
extern uint8_t  ucRegIndex;          /* 最新一帧的数据类型（如 0x53=WIT_ANGLE） */
extern uint16_t usRegDataBuff[4];    /* 最新一帧的 4 个 int16 原始数据 */
extern uint32_t uiRegDataLen;        /* 最新一帧的有效数据个数（通常为 4） */

#ifdef __cplusplus
}
#endif

#endif /* __WIT_C_SDK_H */
