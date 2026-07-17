/* 巡线策略引擎 — 加权法 + 直角弯 + 丢线记忆 */
#include "line_follow.h"
#include "gray.h"

#define Kp        3.0f
#define TURN_PWM  20.0f   /* 直角弯旋转强度 */
#define EDGE_TH   3       /* 单侧 ≥3 判定为直角弯 */

static float  s_last_steer;
static uint8_t s_last_turn;

LineFollow_Result LineFollow_Update(void)
{
    extern GraySensor gs;
    Gray_Update(&gs);

    LineFollow_Result r;
    uint8_t d = Gray_Raw(&gs);

    /* 数字量原始值直接数左右 */
    uint8_t l_cnt = 0, r_cnt = 0;
    for (uint8_t i = 0; i < 4; i++) { if (d & (1 << i)) l_cnt++; }
    for (uint8_t i = 4; i < 8; i++) { if (d & (1 << i)) r_cnt++; }

    if (r_cnt >= EDGE_TH && l_cnt <= 1) {
        /* 右直角弯 */
        r.steer = TURN_PWM;
        r.on_line = 2;
        r.last_turn = 1;
        s_last_steer = TURN_PWM;
        s_last_turn = 1;
    } else if (l_cnt >= EDGE_TH && r_cnt <= 1) {
        /* 左直角弯 */
        r.steer = -TURN_PWM;
        r.on_line = 2;
        r.last_turn = 0;
        s_last_steer = -TURN_PWM;
        s_last_turn = 0;
    } else {
        int8_t pos = Gray_Position(&gs);
        if (pos != 127) {
            r.steer = (float)pos * Kp;
            r.on_line = 1;
            s_last_steer = r.steer;
            s_last_turn = (pos > 0) ? 1 : 0;
        } else {
            r.steer = s_last_steer;
            r.on_line = 0;
        }
        r.last_turn = s_last_turn;
    }
    return r;
}
