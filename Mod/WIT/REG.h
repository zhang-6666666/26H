/* JY901P 传感器寄存器映射表（WIT 维特智能）
 * sReg[REGSIZE] 是 SDK 内部的寄存器镜像数组，sReg[Roll] 即横滚角的原始 int16 值
 * 原始值范围 -32768 ~ +32767，对应 -180° ~ +180°
 */

#ifndef __AHRSREG_H
#define __AHRSREG_H

#ifdef __cplusplus
extern "C" {
#endif

#define REGSIZE 0x90   /* 寄存器总数（144 个，地址 0x00 ~ 0x8F） */

/* ==================== 配置寄存器 ==================== */
#define SAVE        0x00   /* 保存：写 SAVE_PARAM 保存参数，写 SAVE_SWRST 软件复位 */
#define CALSW       0x01   /* 校准模式：NORMAL=正常, CALGYROACC=加速度校准, CALMAGMM=磁场校准 … */
#define RSW         0x02   /* 自动输出内容：位掩码组合 RSW_ANGLE|RSW_ACC|… */
#define RRATE       0x03   /* 数据输出频率：RRATE_02HZ ~ RRATE_200HZ */
#define BAUD        0x04   /* 串口波特率：WIT_BAUD_4800 ~ WIT_BAUD_921600 */

/* ==================== 零偏校准偏移量 ==================== */
#define AXOFFSET    0x05   /* 加速度 X 轴偏移 */
#define AYOFFSET    0x06   /* 加速度 Y 轴偏移 */
#define AZOFFSET    0x07   /* 加速度 Z 轴偏移 */
#define GXOFFSET    0x08   /* 陀螺仪 X 轴偏移 */
#define GYOFFSET    0x09   /* 陀螺仪 Y 轴偏移 */
#define GZOFFSET    0x0a   /* 陀螺仪 Z 轴偏移 */
#define HXOFFSET    0x0b   /* 磁力计 X 轴偏移 */
#define HYOFFSET    0x0c   /* 磁力计 Y 轴偏移 */
#define HZOFFSET    0x0d   /* 磁力计 Z 轴偏移 */

/* ==================== GPIO 端口配置 ==================== */
#define D0MODE      0x0e   /* D0 引脚模式 */
#define D1MODE      0x0f   /* D1 引脚模式 */
#define D2MODE      0x10   /* D2 引脚模式 */
#define D3MODE      0x11   /* D3 引脚模式 */
#define D0PWMH      0x12   /* D0 PWM 高电平时间 */
#define D1PWMH      0x13   /* D1 PWM 高电平时间 */
#define D2PWMH      0x14   /* D2 PWM 高电平时间 */
#define D3PWMH      0x15   /* D3 PWM 高电平时间 */
#define D0PWMT      0x16   /* D0 PWM 周期 */
#define D1PWMT      0x17   /* D1 PWM 周期 */
#define D2PWMT      0x18   /* D2 PWM 周期 */
#define D3PWMT      0x19   /* D3 PWM 周期 */

/* ==================== 通信配置 ==================== */
#define IICADDR     0x1a   /* I2C 设备地址 */
#define LEDOFF      0x1b   /* LED 关闭 */
#define BANDWIDTH   0x1f   /* 低通滤波带宽：BANDWIDTH_256HZ ~ BANDWIDTH_5HZ */

/* ==================== 磁力计量程 ==================== */
#define MAGRANGX    0x1c   /* 磁力计 X 轴量程 */
#define MAGRANGY    0x1d   /* 磁力计 Y 轴量程 */
#define MAGRANGZ    0x1e   /* 磁力计 Z 轴量程 */

/* ==================== 传感器量程 ==================== */
#define GYRORANGE   0x20   /* 陀螺仪量程 */
#define ACCRANGE    0x21   /* 加速度计量程 */

/* ==================== 低功耗 & 算法配置 ==================== */
#define SLEEP       0x22   /* 休眠控制 */
#define ORIENT      0x23   /* 安装方向：ORIENT_HERIZONE=水平, ORIENT_VERTICLE=竖直 */
#define AXIS6       0x24   /* 算法模式：ALGRITHM9=9轴, ALGRITHM6=6轴 */
#define FILTK       0x25   /* 滤波系数 */

/* ==================== GPS 配置 ==================== */
#define GPSBAUD     0x26   /* GPS 模块波特率 */
#define READADDR    0x27   /* 读寄存器命令地址 */
#define BWSCALE     0x28   /* 带宽比例 / 运动阈值 */
#define MOVETHR     0x28   /* 运动检测阈值（同 0x28，别名） */
#define MOVESTA     0x29   /* 运动状态 */

/* ==================== 滤波器参数 ==================== */
#define ACCFILT     0x2A   /* 加速度滤波系数 */
#define GYROFILT    0x2b   /* 陀螺仪滤波系数 */
#define MAGFILT     0x2c   /* 磁力计滤波系数 */
#define POWONSEND   0x2d   /* 上电自动发送 */

/* ==================== 设备信息 ==================== */
#define VERSION     0x2e   /* 固件版本号 */
#define CCBW        0x2f   /* 横滚角方向（顺时针/逆时针） */

/* ==================== 时间 ==================== */
#define YYMM        0x30   /* 年、月 */
#define DDHH        0x31   /* 日、时 */
#define MMSS        0x32   /* 分、秒 */
#define MS          0x33   /* 毫秒（低 8 位） */

/* ==================== 传感器数据（原始值） ==================== */
/* 加速度：单位取决于量程，默认 ±16g → 32768 = 16g */
#define AX          0x34   /* 加速度 X */
#define AY          0x35   /* 加速度 Y */
#define AZ          0x36   /* 加速度 Z */

/* 陀螺仪：单位取决于量程，默认 ±2000°/s → 32768 = 2000°/s */
#define GX          0x37   /* 角速度 X */
#define GY          0x38   /* 角速度 Y */
#define GZ          0x39   /* 角速度 Z */

/* 磁力计 */
#define HX          0x3a   /* 磁场 X */
#define HY          0x3b   /* 磁场 Y */
#define HZ          0x3c   /* 磁场 Z */

/* 角度（核心数据）：原始值 / 32768 * 180 = 角度值 */
#define Roll        0x3d   /* 横滚角  Roll  X 轴角度 */
#define Pitch       0x3e   /* 俯仰角  Pitch Y 轴角度 */
#define Yaw         0x3f   /* 偏航角  Yaw   Z 轴角度 */

/* 温度 */
#define TEMP        0x40   /* 温度：原始值 / 100 = ℃ */

/* 端口状态 */
#define D0Status    0x41   /* D0 端口状态 */
#define D1Status    0x42   /* D1 端口状态 */
#define D2Status    0x43   /* D2 端口状态 */
#define D3Status    0x44   /* D3 端口状态 */

/* 气压 & 高度 */
#define PressureL   0x45   /* 气压低 16 位 */
#define PressureH   0x46   /* 气压高 16 位 */
#define HeightL     0x47   /* 高度低 16 位 */
#define HeightH     0x48   /* 高度高 16 位 */

/* GPS 经纬度 */
#define LonL        0x49   /* 经度低 16 位 */
#define LonH        0x4a   /* 经度高 16 位 */
#define LatL        0x4b   /* 纬度低 16 位 */
#define LatH        0x4c   /* 纬度高 16 位 */
#define GPSHeight   0x4d   /* GPS 高度 */
#define GPSYAW      0x4e   /* GPS 航向角 */

/* 地速 */
#define GPSVL       0x4f   /* 地速低 16 位 */
#define GPSVH       0x50   /* 地速高 16 位 */

/* 四元数 */
#define q0          0x51   /* 四元数 q0 */
#define q1          0x52   /* 四元数 q1 */
#define q2          0x53   /* 四元数 q2 */
#define q3          0x54   /* 四元数 q3 */

/* GPS 卫星信息 */
#define SVNUM       0x55   /* 卫星数量 */
#define PDOP        0x56   /* 位置精度因子 */
#define HDOP        0x57   /* 水平精度因子 */
#define VDOP        0x58   /* 垂直精度因子 */
#define DELAYT      0x59   /* GPS 延迟时间 */

/* 磁场范围 */
#define XMIN        0x5a   /* X 轴磁场最小值 */
#define XMAX        0x5b   /* X 轴磁场最大值 */
#define BATVAL      0x5c   /* 电池电压 */
#define ALARMPIN    0x5d   /* 报警引脚 */
#define YMIN        0x5e   /* Y 轴磁场最小值 */
#define YMAX        0x5f   /* Y 轴磁场最大值 */

/* 陀螺仪零偏自动校准 */
#define GYROZSCALE  0x60   /* 陀螺仪 Z 轴比例 */
#define GYROCALITHR 0x61   /* 陀螺仪校准阈值 */
#define ALARMLEVEL  0x62   /* 报警阈值 */
#define GYROCALTIME 0x63   /* 陀螺仪校准时间 */

/* 参考角度 */
#define REFROLL     0x64   /* 参考横滚角 */
#define REFPITCH    0x65   /* 参考俯仰角 */
#define REFYAW      0x66   /* 参考偏航角 */

/* 高级配置 */
#define GPSTYPE     0x67   /* GPS 类型 */
#define TRIGTIME    0x68   /* 触发时间 */
#define KEY         0x69   /* 解锁寄存器：写 KEY_UNLOCK(0xB588) 后才能写其他寄存器 */
#define WERROR      0x6a   /* 错误码 */
#define TIMEZONE    0x6b   /* 时区 */
#define CALICNT     0x6c   /* 校准计数 */
#define WZCNT       0x6d   /* WZ 计数 */
#define WZTIME      0x6e   /* WZ 时间 */
#define WZSTATIC    0x6f   /* WZ 静态判断 */

/* 传感器类型 */
#define ACCSENSOR   0x70   /* 加速度传感器型号 */
#define GYROSENSOR  0x71   /* 陀螺仪传感器型号 */
#define MAGSENSOR   0x72   /* 磁力计传感器型号 */
#define PRESSENSOR  0x73   /* 气压传感器型号 */
#define MODDELAY    0x74   /* Modbus 延时 */

/* 角度轴 & 比例 */
#define ANGLEAXIS   0x75   /* 角度轴配置 */
#define XRSCALE     0x76   /* X 轴比例 */
#define YRSCALE     0x77   /* Y 轴比例 */
#define ZRSCALE     0x78   /* Z 轴比例 */
#define XREFROLL    0x79   /* X 轴参考横滚角 */
#define YREFPITCH   0x7a   /* Y 轴参考俯仰角 */
#define ZREFYAW     0x7b   /* Z 轴参考偏航角 */
#define ANGXOFFSET  0x7c   /* 角度 X 偏移 */
#define ANGYOFFSET  0x7d   /* 角度 Y 偏移 */
#define ANGZOFFSET  0x7e   /* 角度 Z 偏移 */

/* 设备编号 ID */
#define NUMBERID1   0x7f
#define NUMBERID2   0x80
#define NUMBERID3   0x81
#define NUMBERID4   0x82
#define NUMBERID5   0x83
#define NUMBERID6   0x84

/* 85 系列标定参数 */
#define XA85PSCALE  0x85
#define XA85NSCALE  0x86
#define YA85PSCALE  0x87
#define YA85NSCALE  0x88
#define XA30PSCALE  0x89
#define XA30NSCALE  0x8a
#define YA30PSCALE  0x8b
#define YA30NSCALE  0x8c

/* 芯片 ID */
#define CHIPIDL     0x8D   /* 芯片 ID 低字节 */
#define CHIPIDH     0x8E   /* 芯片 ID 高字节 */
#define REGINITFLAG REGSIZE-1   /* 寄存器初始化标记 */


/* ==================== AXIS6：算法模式 ==================== */
#define ALGRITHM9   0   /* 9 轴算法（加速度+陀螺仪+磁力计） */
#define ALGRITHM6   1   /* 6 轴算法（加速度+陀螺仪） */


/* ==================== CALSW：校准模式 ==================== */
#define NORMAL          0x00   /* 正常模式（不校准） */
#define CALGYROACC      0x01   /* 加速度校准 */
#define CALMAG          0x02   /* 磁场校准 */
#define CALALTITUDE     0x03   /* 高度校准 */
#define CALANGLEZ       0x04   /* Z 轴角度校准 */
#define CALACCL         0x05   /* 加速度左校准 */
#define CALACCR         0x06   /* 加速度右校准 */
#define CALMAGMM        0x07   /* 磁场椭球拟合校准 */
#define CALREFANGLE     0x08   /* 参考角度校准 */
#define CALMAG2STEP     0x09   /* 磁场两步法校准 */
#define CALHEXAHEDRON   0x12   /* 六面校准 */


/* ==================== 数据输出类型标识（帧头 0x55 后的第 2 字节） ==================== */
#define WIT_TIME        0x50   /* 时间 */
#define WIT_ACC         0x51   /* 加速度 */
#define WIT_GYRO        0x52   /* 角速度（陀螺仪） */
#define WIT_ANGLE       0x53   /* 角度（Roll/Pitch/Yaw） */
#define WIT_MAGNETIC    0x54   /* 磁力计 */
#define WIT_DPORT       0x55   /* 数字端口 */
#define WIT_PRESS       0x56   /* 气压 */
#define WIT_GPS         0x57   /* GPS 定位 */
#define WIT_VELOCITY    0x58   /* 速度 */
#define WIT_QUATER      0x59   /* 四元数 */
#define WIT_GSA         0x5A   /* GPS 卫星信息 */
#define WIT_REGVALUE    0x5F   /* 寄存器返回值 */


/* ==================== RSW：自动输出内容（位掩码，可组合） ==================== */
#define RSW_TIME    0x01    /* 输出时间 */
#define RSW_ACC     0x02    /* 输出加速度 */
#define RSW_GYRO    0x04    /* 输出角速度 */
#define RSW_ANGLE   0x08    /* 输出角度（你项目用的就是这个） */
#define RSW_MAG     0x10    /* 输出磁力计 */
#define RSW_PORT    0x20    /* 输出端口状态 */
#define RSW_PRESS   0x40    /* 输出气压 */
#define RSW_GPS     0x80    /* 输出 GPS */
#define RSW_V       0x100   /* 输出速度 */
#define RSW_Q       0x200   /* 输出四元数 */
#define RSW_GSA     0x400   /* 输出 GPS 卫星信息 */
#define RSW_MASK    0xfff   /* 全部输出 */

/* 例如：只输出角度 = RSW_ANGLE (0x08)；同时输出角度+加速度 = RSW_ANGLE | RSW_ACC */


/* ==================== RRATE：数据输出频率 ==================== */
#define RRATE_NONE  0x0d   /* 不自动输出（仅查询模式） */
#define RRATE_02HZ  0x01   /* 0.2 Hz（5 秒一次） */
#define RRATE_05HZ  0x02   /* 0.5 Hz */
#define RRATE_1HZ   0x03   /* 1 Hz */
#define RRATE_2HZ   0x04   /* 2 Hz */
#define RRATE_5HZ   0x05   /* 5 Hz */
#define RRATE_10HZ  0x06   /* 10 Hz */
#define RRATE_20HZ  0x07   /* 20 Hz */
#define RRATE_50HZ  0x08   /* 50 Hz */
#define RRATE_100HZ 0x09   /* 100 Hz */
#define RRATE_125HZ 0x0a   /* 125 Hz（仅 WT931） */
#define RRATE_200HZ 0x0b   /* 200 Hz（最高刷新率） */
#define RRATE_ONCE  0x0c   /* 单次输出 */


/* ==================== BAUD：串口波特率 ==================== */
#define WIT_BAUD_4800     1
#define WIT_BAUD_9600     2
#define WIT_BAUD_19200    3
#define WIT_BAUD_38400    4
#define WIT_BAUD_57600    5
#define WIT_BAUD_115200   6
#define WIT_BAUD_230400   7
#define WIT_BAUD_460800   8
#define WIT_BAUD_921600   9


/* ==================== CAN 波特率 ==================== */
#define CAN_BAUD_1000000   0
#define CAN_BAUD_800000    1
#define CAN_BAUD_500000    2
#define CAN_BAUD_400000    3
#define CAN_BAUD_250000    4
#define CAN_BAUD_200000    5
#define CAN_BAUD_125000    6
#define CAN_BAUD_100000    7
#define CAN_BAUD_80000     8
#define CAN_BAUD_50000     9
#define CAN_BAUD_40000     10
#define CAN_BAUD_20000     11
#define CAN_BAUD_10000     12
#define CAN_BAUD_5000      13
#define CAN_BAUD_3000      14


/* ==================== KEY：寄存器解锁密码 ==================== */
#define KEY_UNLOCK  0xB588   /* 写此值到 KEY 寄存器后才能修改其他寄存器 */


/* ==================== SAVE：保存/复位 ==================== */
#define SAVE_PARAM  0x00     /* 保存当前参数到 Flash */
#define SAVE_SWRST  0xFF     /* 软件复位 */


/* ==================== ORIENT：安装方向 ==================== */
#define ORIENT_HERIZONE  0   /* 水平安装 */
#define ORIENT_VERTICLE  1   /* 垂直安装 */


/* ==================== BANDWIDTH：低通滤波器截止频率 ==================== */
#define BANDWIDTH_256HZ  0   /* 256 Hz（响应最快，噪声最大） */
#define BANDWIDTH_184HZ  1
#define BANDWIDTH_94HZ   2
#define BANDWIDTH_44HZ   3
#define BANDWIDTH_21HZ   4
#define BANDWIDTH_10HZ   5
#define BANDWIDTH_5HZ    6   /* 5 Hz（最平滑，响应最慢） */


#ifdef __cplusplus
}
#endif

#endif
