/* WIT 维特智能传感器 SDK — 核心实现
 *
 * 数据流（正常协议）：
 *   串口收到字节 → WitSerialDataIn() 逐字节拼包 → 校验通过后填入全局变量
 *   → 调用者取 ucRegIndex / usRegDataBuff / uiRegDataLen
 *   → 调用者调用 CopeWitData() 写入 sReg[] 并触发数据回调
 */

#include "wit_c_sdk.h"

/* ==================== 回调函数指针（静态，由注册函数写入） ==================== */
static SerialWrite  p_WitSerialWriteFunc = NULL;  /* 串口发送回调 */
static WitI2cWrite  p_WitI2cWriteFunc    = NULL;  /* I2C 写回调 */
static WitI2cRead   p_WitI2cReadFunc     = NULL;  /* I2C 读回调 */
static CanWrite     p_WitCanWriteFunc    = NULL;  /* CAN 发送回调 */
static RegUpdateCb  p_WitRegUpdateCbFunc = NULL;  /* 数据更新回调（核心） */
static DelaymsCb    p_WitDelaymsFunc     = NULL;  /* 毫秒延时回调 */

/* ==================== SDK 内部状态（静态） ==================== */
static uint8_t  s_ucAddr         = 0xff;          /* 设备地址 */
static uint8_t  s_ucWitDataBuff[WIT_DATA_BUFF_SIZE]; /* 接收缓冲区 */
static uint32_t s_uiWitDataCnt   = 0;             /* 接收字节计数 */
static uint32_t s_uiProtoclo     = 0;             /* 当前协议类型 */
static uint32_t s_uiReadRegIndex = 0;             /* 上一次读寄存器的起始地址 */

/* ==================== SDK 全局变量（外部可读） ==================== */
int16_t  sReg[REGSIZE];               /* 寄存器镜像数组，如 sReg[Roll] = 横滚角 */
uint8_t  ucRegIndex    = 0;           /* 最新一帧的数据类型标识（0x51=加速度, 0x52=陀螺仪, 0x53=角度 …） */
uint16_t usRegDataBuff[4] = {0};      /* 最新一帧的原始数据（最多 4 个 int16） */
uint32_t uiRegDataLen  = 0;           /* 数据个数 */

/* Modbus 功能码 */
#define FuncW 0x06   /* 写单个寄存器 */
#define FuncR 0x03   /* 读多个寄存器 */

/* ==================== CRC16 校验表（Modbus） ==================== */
static const uint8_t __auchCRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};
static const uint8_t __auchCRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};

/* Modbus CRC16 校验 */
static uint16_t __CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint8_t uIndex;
    int i = 0;
    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF;
    for (; i<usDataLen; i++)
    {
        uIndex = uchCRCHi ^ puchMsg[i];
        uchCRCHi = uchCRCLo ^ __auchCRCHi[uIndex];
        uchCRCLo = __auchCRCLo[uIndex] ;
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}

/* 正常协议累加和校验：data[0..len-1] 求和取低 8 位 */
static uint8_t __CaliSum(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t ucCheck = 0;
    for(i=0; i<len; i++) ucCheck += *(data + i);
    return ucCheck;
}

/* 注册串口发送回调 */
int32_t WitSerialWriteRegister(SerialWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}

/* 解析 SDK 内部拼好的数据包，写入 sReg[] 并触发数据更新回调
 * ucIndex: 数据类型（0x53=角度, 0x51=加速度, 0x52=陀螺仪 …）
 * p_data:  4 个 int16 原始值
 * uiLen:   有效数据个数（正常协议恒为 4）
 */
void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
{
    uint32_t uiReg1 = 0, uiReg2 = 0, uiReg1Len = 0, uiReg2Len = 0;
    uint16_t *p_usReg1Val = p_data;       /* 前 3 个 int16 → 主数据（如 Roll/Pitch/Yaw） */
    uint16_t *p_usReg2Val = p_data+3;     /* 第 4 个 int16 → 辅数据（如 VERSION） */

    uiReg1Len = 4;
    switch(ucIndex)
    {
        case WIT_ACC:   uiReg1 = AX;    uiReg1Len = 3;  uiReg2 = TEMP;  uiReg2Len = 1;  break;
        case WIT_ANGLE: uiReg1 = Roll;  uiReg1Len = 3;  uiReg2 = VERSION;  uiReg2Len = 1;  break;
        case WIT_TIME:  uiReg1 = YYMM;	break;
        case WIT_GYRO:  uiReg1 = GX;  uiLen = 3;break;
        case WIT_MAGNETIC: uiReg1 = HX;  uiLen = 3;break;
        case WIT_DPORT: uiReg1 = D0Status;  break;
        case WIT_PRESS: uiReg1 = PressureL;  break;
        case WIT_GPS:   uiReg1 = LonL;  break;
        case WIT_VELOCITY: uiReg1 = GPSHeight;  break;
        case WIT_QUATER:    uiReg1 = q0;  break;
        case WIT_GSA:   uiReg1 = SVNUM;  break;
        case WIT_REGVALUE:  uiReg1 = s_uiReadRegIndex;  break;  /* 读寄存器返回 */
        default:
            return ;
    }
    if(uiLen == 3)
    {
        uiReg1Len = 3;
        uiReg2Len = 0;
    }
    /* 把原始数据拷贝到 sReg 并触发回调，告诉上层哪些寄存器更新了 */
    if(uiReg1Len)
    {
        memcpy(&sReg[uiReg1], p_usReg1Val, uiReg1Len<<1);
        p_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
    }
    if(uiReg2Len)
    {
        memcpy(&sReg[uiReg2], p_usReg2Val, uiReg2Len<<1);
        p_WitRegUpdateCbFunc(uiReg2, uiReg2Len);
    }
}

/* 核心：逐字节喂给 SDK 状态机，内部自动拼包、校验、解析
 *
 * 正常协议（WIT_PROTOCOL_NORMAL）：
 *   帧格式：0x55 + 数据类型(1B) + 数据(8B, 4×int16) + 累加和(1B) = 共 11 字节
 *   流程：
 *     1. 找 0x55 帧头，不是则丢弃直到找到
 *     2. 收到 11 字节后做累加和校验
 *     3. 校验失败：丢弃首字节，滑动窗口继续搜索
 *     4. 校验通过：把 8 字节数据拆成 4 个 int16，填入全局变量 usRegDataBuff
 *     5. 注意：正常协议不直接调 CopeWitData()，由调用者在外部调用
 *
 * Modbus 协议（WIT_PROTOCOL_MODBUS）：
 *   读寄存器返回格式：地址 + 0x03 + 字节数 + 数据 + CRC16
 *   校验通过后直接写入 sReg 并触发回调
 */
void WitSerialDataIn(uint8_t ucData)
{
    uint16_t usCRC16, usTemp, i, usData[4];
    uint8_t ucSum;

    if(p_WitRegUpdateCbFunc == NULL)return ;
    s_ucWitDataBuff[s_uiWitDataCnt++] = ucData;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:   /* 正常协议：0x55 帧头 + 累加和校验 */
            if(s_ucWitDataBuff[0] != 0x55)      /* 未找到帧头，丢弃首字节继续搜索 */
            {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                return ;
            }
            if(s_uiWitDataCnt >= 11)            /* 收到完整一帧（11 字节） */
            {
                ucSum = __CaliSum(s_ucWitDataBuff, 10);     /* 前 10 字节累加和 */
                if(ucSum != s_ucWitDataBuff[10])            /* 校验失败：丢首字节滑动窗口 */
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                /* 校验通过：8 字节数据 → 4 个 int16（小端序） */
                usData[0] = ((uint16_t)s_ucWitDataBuff[3] << 8) | (uint16_t)s_ucWitDataBuff[2];
                usData[1] = ((uint16_t)s_ucWitDataBuff[5] << 8) | (uint16_t)s_ucWitDataBuff[4];
                usData[2] = ((uint16_t)s_ucWitDataBuff[7] << 8) | (uint16_t)s_ucWitDataBuff[6];
                usData[3] = ((uint16_t)s_ucWitDataBuff[9] << 8) | (uint16_t)s_ucWitDataBuff[8];
                /* 注意：这里不调 CopeWitData()，由调用者在外部调用 */
//                CopeWitData(s_ucWitDataBuff[1], usData, 4);
                s_uiWitDataCnt = 0;
                ucRegIndex = s_ucWitDataBuff[1];       /* 数据类型（如 0x53） */
                memcpy(usRegDataBuff, usData, 8);      /* 4 个 int16 → 8 字节 */
                uiRegDataLen = 4;
            }
        break;
        case WIT_PROTOCOL_MODBUS:   /* Modbus 协议：CRC16 校验 */
            if(s_uiWitDataCnt > 2)
            {
                if(s_ucWitDataBuff[1] != FuncR)        /* 只处理读寄存器返回（功能码 0x03） */
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                if(s_uiWitDataCnt < (s_ucWitDataBuff[2] + 5))return ;  /* 还没收完 */
                usTemp = ((uint16_t)s_ucWitDataBuff[s_uiWitDataCnt-2] << 8) | s_ucWitDataBuff[s_uiWitDataCnt-1];
                usCRC16 = __CRC16(s_ucWitDataBuff, s_uiWitDataCnt-2);
                if(usTemp != usCRC16)                   /* CRC 校验失败 */
                {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return ;
                }
                /* Modbus 直接写入 sReg 并触发回调 */
                usTemp = s_ucWitDataBuff[2] >> 1;
                for(i = 0; i < usTemp; i++)
                {
                    sReg[i+s_uiReadRegIndex] = ((uint16_t)s_ucWitDataBuff[(i<<1)+3] << 8) | s_ucWitDataBuff[(i<<1)+4];
                }
                p_WitRegUpdateCbFunc(s_uiReadRegIndex, usTemp);
                s_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_CAN:
        case WIT_PROTOCOL_I2C:
        s_uiWitDataCnt = 0;
        break;
    }
    if(s_uiWitDataCnt == WIT_DATA_BUFF_SIZE)s_uiWitDataCnt = 0;  /* 缓冲区溢出保护 */
}

/* 注册 I2C 读写回调 */
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func)
{
    if(!write_func)return WIT_HAL_INVAL;
    if(!read_func)return WIT_HAL_INVAL;
    p_WitI2cWriteFunc = write_func;
    p_WitI2cReadFunc = read_func;
    return WIT_HAL_OK;
}

/* 注册 CAN 发送回调 */
int32_t WitCanWriteRegister(CanWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    p_WitCanWriteFunc = Write_func;
    return WIT_HAL_OK;
}

/* 喂一帧 CAN 数据（8 字节）给 SDK */
void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen)
{
    uint16_t usData[3];
    if(p_WitRegUpdateCbFunc == NULL)return ;
    if(ucLen < 8)return ;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_CAN:
            if(ucData[0] != 0x55)return ;
            usData[0] = ((uint16_t)ucData[3] << 8) | ucData[2];
            usData[1] = ((uint16_t)ucData[5] << 8) | ucData[4];
            usData[2] = ((uint16_t)ucData[7] << 8) | ucData[6];
            CopeWitData(ucData[1], usData, 3);  /* CAN 协议直接调用 CopeWitData */
            break;
        case WIT_PROTOCOL_NORMAL:
        case WIT_PROTOCOL_MODBUS:
        case WIT_PROTOCOL_I2C:
            break;
    }
}

/* 注册数据更新回调 */
int32_t WitRegisterCallBack(RegUpdateCb update_func)
{
    if(!update_func)return WIT_HAL_INVAL;
    p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}

/* 向传感器写一个寄存器
 * 正常协议帧：0xFF 0xAA + 寄存器地址(1B) + 值(2B, 小端)
 * Modbus 帧： 地址 + 0x06 + 寄存器(2B) + 值(2B) + CRC16(2B)
 */
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData)
{
    uint16_t usCRC;
    uint8_t ucBuff[8];
    if(uiReg >= REGSIZE)return WIT_HAL_INVAL;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_WitSerialWriteFunc(ucBuff, 5);    /* 5 字节写命令 */
            break;
        case WIT_PROTOCOL_MODBUS:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncW;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = usData >> 8;
            ucBuff[5] = usData & 0xff;
            usCRC = __CRC16(ucBuff, 6);
            ucBuff[6] = usCRC >> 8;
            ucBuff[7] = usCRC & 0xff;
            p_WitSerialWriteFunc(ucBuff, 8);    /* 8 字节写命令 */
            break;
        case WIT_PROTOCOL_CAN:
            if(p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_WitI2cWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = usData & 0xff;
            ucBuff[1] = usData >> 8;
            if(p_WitI2cWriteFunc(s_ucAddr << 1, uiReg, ucBuff, 2) != 1)
            {
                //printf("i2c write fail\r\n");
            }
        break;
        default:
            return WIT_HAL_INVAL;
    }
    return WIT_HAL_OK;
}

/* 向传感器读若干寄存器
 * 正常协议帧：0xFF 0xAA 0x27 + 起始地址(2B, 小端)
 * 传感器会返回数据，由 WitSerialDataIn 接收解析
 */
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum)
{
    uint16_t usTemp, i;
    uint8_t ucBuff[8];
    if((uiReg + uiReadNum) >= REGSIZE)return WIT_HAL_INVAL;
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(uiReadNum > 4)return WIT_HAL_INVAL;      /* 正常协议一次最多读 4 个 */
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;                            /* 0x27 = 读寄存器命令 */
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if((usTemp + 5) > WIT_DATA_BUFF_SIZE)return WIT_HAL_NOMEM;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncR;                           /* 功能码 0x03 = 读寄存器 */
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = uiReadNum >> 8;
            ucBuff[5] = uiReadNum & 0xff;
            usTemp = __CRC16(ucBuff, 6);
            ucBuff[6] = usTemp >> 8;
            ucBuff[7] = usTemp & 0xff;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(uiReadNum > 3)return WIT_HAL_INVAL;
            if(p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_WitI2cReadFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if(WIT_DATA_BUFF_SIZE < usTemp)return WIT_HAL_NOMEM;
            if(p_WitI2cReadFunc(s_ucAddr << 1, uiReg, s_ucWitDataBuff, usTemp) == 1)
            {
                if(p_WitRegUpdateCbFunc == NULL)return WIT_HAL_EMPTY;
                for(i = 0; i < uiReadNum; i++)
                {
                    sReg[i+uiReg] = ((uint16_t)s_ucWitDataBuff[(i<<1)+1] << 8) | s_ucWitDataBuff[i<<1];
                }
                p_WitRegUpdateCbFunc(uiReg, uiReadNum);
            }
            break;
        default:
            return WIT_HAL_INVAL;
    }
    s_uiReadRegIndex = uiReg;   /* 记录本次读取的起始寄存器，供 CopeWitData 使用 */
    return WIT_HAL_OK;
}

/* SDK 初始化：设置协议类型和设备地址，清零接收状态 */
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
    if(uiProtocol > WIT_PROTOCOL_I2C)return WIT_HAL_INVAL;
    s_uiProtoclo = uiProtocol;
    s_ucAddr = ucAddr;
    s_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}

/* SDK 反初始化：清空所有回调指针和状态 */
void WitDeInit(void)
{
    p_WitSerialWriteFunc = NULL;
    p_WitI2cWriteFunc = NULL;
    p_WitI2cReadFunc = NULL;
    p_WitCanWriteFunc = NULL;
    p_WitRegUpdateCbFunc = NULL;
    s_ucAddr = 0xff;
    s_uiWitDataCnt = 0;
    s_uiProtoclo = 0;
}

/* 注册毫秒延时回调 */
int32_t WitDelayMsRegister(DelaymsCb delayms_func)
{
    if(!delayms_func)return WIT_HAL_INVAL;
    p_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

/* 范围检查工具 */
char CheckRange(short sTemp, short sMin, short sMax)
{
    if ((sTemp>=sMin)&&(sTemp<=sMax)) return 1;
    else return 0;
}

/* ==================== 传感器配置函数 ==================== */
/* 以下函数统一流程：解锁 KEY 寄存器 → 写目标寄存器 → 中间延时等待传感器处理 */

/* 开始加速度校准（需水平放置设备） */
int32_t WitStartAccCali(void)
{
    if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)      return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
    else ;
    if(WitWriteReg(CALSW, CALGYROACC) != WIT_HAL_OK)    return  WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

/* 停止加速度校准并保存参数 */
int32_t WitStopAccCali(void)
{
    if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)        return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
    else ;
    if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)     return  WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

/* 开始磁场校准 */
int32_t WitStartMagCali(void)
{
    if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)      return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
    else ;
    if(WitWriteReg(CALSW, CALMAGMM) != WIT_HAL_OK)      return  WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

/* 停止磁场校准 */
int32_t WitStopMagCali(void)
{
    if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)      return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
    else ;
    if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)        return  WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

/* 设置串口波特率 */
int32_t WitSetUartBaud(int32_t uiBaudIndex)
{
    if(!CheckRange(uiBaudIndex, WIT_BAUD_4800, WIT_BAUD_230400)) return WIT_HAL_INVAL;
    if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)      return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
    else ;
    if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)    return  WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

/* 设置 CAN 波特率 */
int32_t WitSetCanBaud(int32_t uiBaudIndex)
{
    if(!CheckRange(uiBaudIndex, CAN_BAUD_1000000, CAN_BAUD_3000)) return WIT_HAL_INVAL;
    if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)      return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
    else ;
    if(WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)    return  WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

/* 设置低通滤波带宽（值越小滤波越强，数据越平滑但响应越慢） */
int32_t WitSetBandwidth(int32_t uiBaudWidth)
{
    if(!CheckRange(uiBaudWidth, BANDWIDTH_256HZ, BANDWIDTH_5HZ)) return WIT_HAL_INVAL;
    if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)      return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
    else ;
    if(WitWriteReg(BANDWIDTH, uiBaudWidth) != WIT_HAL_OK) return  WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

/* 设置数据输出频率 */
int32_t WitSetOutputRate(int32_t uiRate)
{
    if(!CheckRange(uiRate, RRATE_02HZ, RRATE_NONE)) return WIT_HAL_INVAL;
    if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)      return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
    else ;
    if(WitWriteReg(RRATE, uiRate) != WIT_HAL_OK)        return  WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

/* 设置自动输出内容（如 RSW_ANGLE=只输出角度，减小数据量提高刷新率） */
int32_t WitSetContent(int32_t uiRsw)
{
    if(!CheckRange(uiRsw, RSW_TIME, RSW_MASK)) return WIT_HAL_INVAL;
    if(WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)      return  WIT_HAL_ERROR;
    if(s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(1);
    else ;
    if(WitWriteReg(RSW, uiRsw) != WIT_HAL_OK)           return  WIT_HAL_ERROR;
    return WIT_HAL_OK;
}
