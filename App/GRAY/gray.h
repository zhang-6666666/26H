/* 8 路灰度传感器驱动 — GPIO 数字输入 + 加权位置
   物理顺序：从左到右 0 1 2 3 4 5 6 7
*/
#ifndef GRAY_H
#define GRAY_H

#include <stdint.h>


typedef struct {
    uint8_t  digital;         /* bit[0..7] 二值化，1=黑线 */
} GraySensor;

void    Gray_Update(GraySensor *sensor);
int8_t  Gray_Position(const GraySensor *sensor);   /* -7~+7，0 居中，127=丢线 */

extern GraySensor gs;

#endif
