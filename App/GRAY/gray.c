/* 8 路灰度传感器驱动 — GPIO 数字输入 + 加权位置
   物理排列：从左到右 0 1 2 3 4 5 6 7
*/

#include "gray.h"
#include "main.h"

GraySensor gs;

/* ---- 位置权重：左负右正，8 路均匀分布 ---- */
/*          通道:    0    1    2    3     4    5    6    7   */
static const int8_t s_weight[8] = { -7, -5, -3, -1,  1,  3,  5,  7 };

/* ---- 每路对应的 GPIO 端口和引脚 ---- */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} LinePin;

static const LinePin s_line[8] = {
    { LINE_0_GPIO_Port, LINE_0_Pin },   /* 最左  */
    { LINE_1_GPIO_Port, LINE_1_Pin },
    { LINE_2_GPIO_Port, LINE_2_Pin },
    { LINE_3_GPIO_Port, LINE_3_Pin },
    { LINE_4_GPIO_Port, LINE_4_Pin },
    { LINE_5_GPIO_Port, LINE_5_Pin },
    { LINE_6_GPIO_Port, LINE_6_Pin },
    { LINE_7_GPIO_Port, LINE_7_Pin },   /* 最右  */
};

/* ================================================================ */

void Gray_Update(GraySensor *sensor)
{
    uint8_t d = 0;

    for (uint8_t i = 0; i < 8; i++) {
        /* 低电平 = 黑线（传感器模块输出低有效） */
        if (HAL_GPIO_ReadPin(s_line[i].port, s_line[i].pin) == GPIO_PIN_RESET) {
            d |= (1 << i);
        }
    }
    sensor->digital = d;
}

int8_t Gray_Position(const GraySensor *sensor)
{
    int8_t  sum = 0;
    uint8_t cnt = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if (sensor->digital & (1 << i)) {
            sum += s_weight[i];
            cnt++;
        }
    }

    return (cnt == 0) ? 127 : (sum / cnt);
}

