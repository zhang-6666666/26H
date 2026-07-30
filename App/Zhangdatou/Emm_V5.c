#include "Emm_V5.h"
#include "motor_bujin.h"

/**********************************************************
*** Emm_V5.0 闭环步进驱动器
*** 编写作者：ZHANGDATOU
*** 硬件支持：张大头闭环步进驱动板
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN博客：https://blog.csdn.net/zhangdatou666
*** QQ交流群：262438510
**********************************************************/

/**
  * @brief  当前位置清零
  * @param  addr  驱动器地址
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 指令长度
  cmd[2] =  0x6D;                       // 命令码
  cmd[3] =  0x6B;                       // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief  解除堵转保护
  * @param  addr  驱动器地址
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 指令长度
  cmd[2] =  0x52;                       // 命令码
  cmd[3] =  0x6B;                       // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief  读取系统参数
  * @param  addr  驱动器地址
  * @param  s     要读取的参数类型
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                              // 选择要读取的参数
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;   // 固件版本和硬件版本
    case S_RL   : cmd[i] = 0x20; ++i; break;   // 读取负载电阻值
    case S_PID  : cmd[i] = 0x21; ++i; break;   // 读取PID参数
    case S_VBUS : cmd[i] = 0x24; ++i; break;   // 读取总线电压
    case S_CPHA : cmd[i] = 0x27; ++i; break;   // 读取相电流
    case S_ENCL : cmd[i] = 0x31; ++i; break;   // 读取编码器校准值
    case S_TPOS : cmd[i] = 0x33; ++i; break;   // 读取目标位置角度
    case S_VEL  : cmd[i] = 0x35; ++i; break;   // 读取实时转速
    case S_CPOS : cmd[i] = 0x36; ++i; break;   // 读取实时位置角度
    case S_PERR : cmd[i] = 0x37; ++i; break;   // 读取位置误差角度
    case S_FLAG : cmd[i] = 0x3A; ++i; break;   // 读取使能/回零/堵转状态标志
    case S_ORG  : cmd[i] = 0x3B; ++i; break;   // 读取回零/回零失败状态标志
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break; // 读取驱动参数
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break; // 读取系统状态
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节

  // 发送指令
  usart_SendCmd(cmd, i);
}

/**
  * @brief  修改开环/闭环控制模式
  * @param  addr       驱动器地址
  * @param  svF        是否存储标志：false=不存储，true=存储
  * @param  ctrl_mode  控制模式：0=关闭电机(脱机)，1=开环模式，2=闭环模式，3=En端口控制电机脱机、Dir端口控制正反转
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 指令长度
  cmd[2] =  0x69;                       // 命令码
  cmd[3] =  svF;                        // 是否存储：false=不存储，true=存储
  cmd[4] =  ctrl_mode;                  // 控制模式：0=脱机，1=开环，2=闭环，3=En/Dir端口控制
  cmd[5] =  0x6B;                       // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 6);
}

/**
  * @brief  电机使能控制
  * @param  addr   驱动器地址
  * @param  state  使能状态：true=使能电机，false=关闭电机
  * @param  snF    多机同步标志：false=立即生效，true=等待同步触发
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 指令长度
  cmd[2] =  0xAB;                       // 命令码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 6);
}

/**
  * @brief  速度模式控制
  * @param  addr  驱动器地址
  * @param  dir   方向：0=CW(顺时针)，非0=CCW(逆时针)
  * @param  vel   目标速度，范围 0~5000 RPM
  * @param  acc   加速度，范围 0~255（注意：0表示直接启动，无加速过程）
  * @param  snF   多机同步标志：false=立即生效，true=等待同步触发
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 指令长度
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM) 高8位
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM) 低8位
  cmd[5] =  acc;                        // 加速度（0=直接启动）
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 8);
}

/**
  * @brief  位置模式控制
  * @param  addr  驱动器地址
  * @param  dir   方向：0=CW(顺时针)，非0=CCW(逆时针)
  * @param  vel   目标速度(RPM)，范围 0~5000
  * @param  acc   加速度，范围 0~255（注意：0表示直接启动）
  * @param  clk   脉冲数，范围 0~(2^32-1)
  * @param  raF   相对/绝对标志：false=相对运动，true=绝对位置运动
  * @param  snF   多机同步标志：false=立即生效，true=等待同步触发
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 指令长度
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM) 高8位
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM) 低8位
  cmd[5]  =  acc;                       // 加速度（0=直接启动）
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24~bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16~bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8~bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0~bit7)
  cmd[10] =  raF;                       // false=相对运动，true=绝对位置运动
  cmd[11] =  snF;                       // 多机同步运动标志
  cmd[12] =  0x6B;                      // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 13);
}

/**
  * @brief  立即停止（所有控制模式通用）
  * @param  addr  驱动器地址
  * @param  snF   多机同步标志：false=立即生效，true=等待同步触发
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 指令长度
  cmd[2] =  0x98;                       // 命令码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief  多机同步运动触发
  * @param  addr  驱动器地址
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 指令长度
  cmd[2] =  0x66;                       // 命令码
  cmd[3] =  0x6B;                       // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief  设置当前位置为原点（单圈绝对零点）
  * @param  addr  驱动器地址
  * @param  svF   是否存储标志：false=不存储，true=存储
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 指令长度
  cmd[2] =  0x88;                       // 命令码
  cmd[3] =  svF;                        // false=不存储，true=存储
  cmd[4] =  0x6B;                       // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief  修改回零参数
  * @param  addr   驱动器地址
  * @param  svF    是否存储标志：false=不存储，true=存储
  * @param  o_mode 回零模式：0=单圈就近回零，1=单圈方向回零，2=单圈限位碰撞回零，3=单圈限位开关回零
  * @param  o_dir  回零方向：0=CW(顺时针)，非0=CCW(逆时针)
  * @param  o_vel  回零速度，单位 RPM(转/分钟)
  * @param  o_tm   回零超时时间，单位 毫秒
  * @param  sl_vel 单圈限位碰撞回零转速，单位 RPM
  * @param  sl_ma  单圈限位碰撞回零电流，单位 mA(毫安)
  * @param  sl_ms  单圈限位碰撞回零时间，单位 ms(毫秒)
  * @param  potF   上电自动触发回零：false=不使能，true=使能
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 指令长度
  cmd[2] =  0xAE;                       // 命令码
  cmd[3] =  svF;                        // false=不存储，true=存储
  cmd[4] =  o_mode;                     // 回零模式：0=就近回零，1=方向回零，2=限位碰撞回零，3=限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM) 高8位
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM) 低8位
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24~bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16~bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8~bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0~bit7)
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 限位碰撞回零转速(RPM) 高8位
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 限位碰撞回零转速(RPM) 低8位
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 限位碰撞回零电流(mA) 高8位
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 限位碰撞回零电流(mA) 低8位
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 限位碰撞回零时间(ms) 高8位
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 限位碰撞回零时间(ms) 低8位
  cmd[18] =  potF;                      // 上电自动回零：false=不使能，true=使能
  cmd[19] =  0x6B;                      // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 20);
}

/**
  * @brief  触发回零
  * @param  addr   驱动器地址
  * @param  o_mode 回零模式：0=单圈就近回零，1=单圈方向回零，2=单圈限位碰撞回零，3=单圈限位开关回零
  * @param  snF    多机同步标志：false=立即生效，true=等待同步触发
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 指令长度
  cmd[2] =  o_mode;                     // 回零模式：0=就近回零，1=方向回零，2=限位碰撞回零，3=限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief  强制中断并退出回零
  * @param  addr  驱动器地址
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 组装指令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 指令长度
  cmd[2] =  0x48;                       // 命令码
  cmd[3] =  0x6B;                       // 校验字节

  // 发送指令
  usart_SendCmd(cmd, 4);
}
