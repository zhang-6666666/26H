/* 通用 PID 控制器 — 纯算法，零依赖，可移植到任意平台 */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;       /* 系数 */
    float target;           /* 控制目标 */
    float integral;         /* 积分累加（内部） */
    float prev_err;         /* 上次误差（微分用） */
    float out_min, out_max; /* 输出限幅 */
} PID_t;

/* 初始化：设系数和输出范围 */
void pid_init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max);

/* 设定目标值 */
void pid_target(PID_t *p, float tgt);

/* 清零积分（模式切换时用） */
void pid_reset(PID_t *p);

/* 核心：传入测量值和采样间隔(秒)，返回控制量 */
float pid_compute(PID_t *p, float measurement, float dt);

#endif
