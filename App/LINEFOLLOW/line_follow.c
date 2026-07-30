/* 巡线策略 — 重心法 PD + 丢线保持 */
#include "line_follow.h"
#include "gray.h"

#define KP  4.0f
#define KD  0.07f        /* KD 越大抑制震荡越强，但太大响应迟钝 */

static float s_last_steer;
static int8_t s_last_pos = 127;

void LineFollow_Reset(void)
{
    s_last_steer = 0.0f;
    s_last_pos   = 127;
}

float LineFollow_Update(void)
{
    Gray_Update(&gs);

    int8_t pos = Gray_Position(&gs);
    float steer;

    if (pos != 127) {
        steer = (float)pos * KP;
        if (s_last_pos != 127) {                    /* 上一帧有线才加 D */
            steer += (float)(pos - s_last_pos) * KD;
        }
        s_last_steer = steer;
        s_last_pos   = pos;
    } else {
        steer = s_last_steer;
        s_last_pos = 127;
    }
    return steer;
}
