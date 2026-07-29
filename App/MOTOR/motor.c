/* TB6612 双路电机驱动 — 方向控制 + PWM 占空比 */
#include "motor.h"

Motor motor_left;
Motor motor_right;

static void motor_dir(Motor *m, uint8_t cw)
{
    if (cw) {
        HAL_GPIO_WritePin(m->cfg.ain1_port, m->cfg.ain1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->cfg.ain2_port, m->cfg.ain2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(m->cfg.ain1_port, m->cfg.ain1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->cfg.ain2_port, m->cfg.ain2_pin, GPIO_PIN_SET);
    }
}

void Motor_Init(Motor *m, const Motor_Config *cfg)
{
    m->cfg = *cfg;

    Motor_Coast(m);

    HAL_TIM_PWM_Start(m->cfg.htim, m->cfg.pwm_channel);
    __HAL_TIM_SET_COMPARE(m->cfg.htim, m->cfg.pwm_channel, 0);
}

void Motor_Run(Motor *m, int16_t permil)
{
    if (permil == 0) { Motor_Coast(m); return; }

    motor_dir(m, permil > 0);

    uint16_t ccr = (uint16_t)((permil > 0 ? permil : -permil) * m->cfg.pwm_period / 1000);
    __HAL_TIM_SET_COMPARE(m->cfg.htim, m->cfg.pwm_channel, ccr);
}

void Motor_Coast(Motor *m)
{
    HAL_GPIO_WritePin(m->cfg.ain1_port, m->cfg.ain1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->cfg.ain2_port, m->cfg.ain2_pin, GPIO_PIN_RESET);
}

void Motor_Brake(Motor *m)
{
    HAL_GPIO_WritePin(m->cfg.ain1_port, m->cfg.ain1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->cfg.ain2_port, m->cfg.ain2_pin, GPIO_PIN_SET);
}
