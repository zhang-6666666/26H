#ifndef __EMM_V5_H
#define __EMM_V5_H

#include <stdint.h>
#include <stdbool.h>

/**********************************************************
*** Emm_V5.0 闭环步进驱动器
*** 编写作者：ZHANGDATOU
*** 硬件支持：张大头闭环步进驱动板
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN博客：https://blog.csdn.net/zhangdatou666
*** QQ交流群：262438510
**********************************************************/

#define   ABS(x)   ((x) > 0 ? (x) : -(x))

/* 系统参数类型枚举 */
typedef enum {
  S_VER   = 0,      /* 固件版本和硬件版本 */
  S_RL    = 1,      /* 读取负载电阻值 */
  S_PID   = 2,      /* 读取PID参数 */
  S_VBUS  = 3,      /* 读取总线电压 */
  S_CPHA  = 5,      /* 读取相电流 */
  S_ENCL  = 7,      /* 读取编码器校准值 */
  S_TPOS  = 8,      /* 读取目标位置角度 */
  S_VEL   = 9,      /* 读取实时转速 */
  S_CPOS  = 10,     /* 读取实时位置角度 */
  S_PERR  = 11,     /* 读取位置误差角度 */
  S_FLAG  = 13,     /* 读取使能/回零/堵转状态标志 */
  S_Conf  = 14,     /* 读取驱动配置参数 */
  S_State = 15,     /* 读取系统运行状态 */
  S_ORG   = 16,     /* 读取回零/回零失败状态标志 */
} SysParams_t;


/**********************************************************
*** 各函数参数详细说明请参见对应 .c 文件中的注释
**********************************************************/
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);                                  // 当前位置清零
void Emm_V5_Reset_Clog_Pro(uint8_t addr);                                        // 解除堵转保护
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);                        // 读取系统参数
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);         // 修改开环/闭环控制模式
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);                      // 电机使能控制
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);               // 速度模式控制
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF); // 位置模式控制
void Emm_V5_Stop_Now(uint8_t addr, bool snF);                                    // 立即停止
void Emm_V5_Synchronous_motion(uint8_t addr);                                    // 多机同步运动触发
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);                                // 设置当前位置为原点
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // 修改回零参数
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);       // 触发回零
void Emm_V5_Origin_Interrupt(uint8_t addr);                                      // 强制中断退出回零

#endif
