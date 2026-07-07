/* TB6612 双路电机驱动 — 方向控制 + PWM 占空比 */
#include "motor.h"

/* ===================== TB6612 引脚定义 ===================== */
#define AIN1_PORT   GPIOB
#define AIN1_PIN    GPIO_PIN_13
#define AIN2_PORT   GPIOB
#define AIN2_PIN    GPIO_PIN_12
#define BIN1_PORT   GPIOB
#define BIN1_PIN    GPIO_PIN_14
#define BIN2_PORT   GPIOB
#define BIN2_PIN    GPIO_PIN_15

#define PWM_CH_A    TIM_CHANNEL_4   /* PA11 */
#define PWM_CH_B    TIM_CHANNEL_1   /* PA8  */

/* ===================== 内部状态 ===================== */
static TIM_HandleTypeDef *motor_htim;   /* TIM1 句柄 */
static const uint16_t PWM_PERIOD = 999; /* 与 CubeMX ARR 一致 */

/* ===================== 方向控制底层 ===================== */
static void motor_a_dir(uint8_t cw)
{
    if (cw) { /* IN1=H IN2=L → CW */
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
    } else {  /* IN1=L IN2=H → CCW */
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
    }
}

static void motor_b_dir(uint8_t cw)
{
    if (cw) {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
    }
}

/* ===================== API 实现 ===================== */

void motor_init(TIM_HandleTypeDef *htim)
{
    motor_htim = htim;

    /* 初始方向设为停止（滑行） */
    motor_a_coast();
    motor_b_coast();

    /* 启动双通道 PWM 输出，初始占空比 0 */
    HAL_TIM_PWM_Start(htim, PWM_CH_A);
    __HAL_TIM_SET_COMPARE(htim, PWM_CH_A, 0);

    HAL_TIM_PWM_Start(htim, PWM_CH_B);
    __HAL_TIM_SET_COMPARE(htim, PWM_CH_B, 0);
}

void motor_a_run(int16_t permil)
{
    if (permil == 0) { motor_a_coast(); return; }

    /* 方向：正值 CW，负值 CCW */
    motor_a_dir(permil > 0);

    /* 占空比：|permil| / 1000 × 999 */
    uint16_t ccr = (uint16_t)((permil > 0 ? permil : -permil) * PWM_PERIOD / 1000);
    __HAL_TIM_SET_COMPARE(motor_htim, PWM_CH_A, ccr);
}

void motor_b_run(int16_t permil)
{
    if (permil == 0) { motor_b_coast(); return; }

    motor_b_dir(permil > 0);

    uint16_t ccr = (uint16_t)((permil > 0 ? permil : -permil) * PWM_PERIOD / 1000);
    __HAL_TIM_SET_COMPARE(motor_htim, PWM_CH_B, ccr);
}

void motor_a_coast(void)
{
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
}

void motor_b_coast(void)
{
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
}

void motor_a_brake(void)
{
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
}

void motor_b_brake(void)
{
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
}
