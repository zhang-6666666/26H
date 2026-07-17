/* 8 路灰度传感器驱动 — ADC 多路复用读取 + 二值化 + 加权位置 */
#include "gray.h"
#include "adc.h"

GraySensor gs;

/* 引脚定义 */
#define AD0_PORT   GPIOB
#define AD0_PIN    GPIO_PIN_0
#define AD1_PORT   GPIOB
#define AD1_PIN    GPIO_PIN_1
#define AD2_PORT   GPIOB
#define AD2_PIN    GPIO_PIN_4

/* 物理位置权重：左负右正 */
static const int8_t s_w[8] = { -8, -6, -4, -1, 1, 4, 6, 8 };

static void set_addr(uint8_t ch)
{
    HAL_GPIO_WritePin(AD0_PORT, AD0_PIN, (ch & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(AD1_PORT, AD1_PIN, (ch & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(AD2_PORT, AD2_PIN, (ch & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint16_t read_adc(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
        return HAL_ADC_GetValue(&hadc1);
    return 0;
}

void Gray_Init(GraySensor *gs) { (void)gs; }

void Gray_Update(GraySensor *gs)
{
    uint8_t d = 0;

    for (uint8_t i = 0; i < 8; i++) {
        set_addr(i);
        uint32_t sum = 0;
        for (uint8_t j = 0; j < 8; j++) sum += read_adc();
        gs->raw[i] = (uint16_t)(sum / 8);

        /* 二值化：低于阈值 → 黑线 */
        if (gs->raw[i] < (GRAY_BLACK_DEFAULT + GRAY_WHITE_DEFAULT) / 2)
            d |= (1 << i);
    }
    gs->digital = d;
}

int8_t Gray_Position(const GraySensor *gs)
{
    int8_t sum = 0, cnt = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if (gs->digital & (1 << i)) {
            sum += s_w[i];
            cnt++;
        }
    }

    return (cnt == 0) ? 127 : (sum / cnt);
}

uint8_t Gray_Raw(const GraySensor *gs) { return gs->digital; }
