/* SSD1306 OLED 128×64 I2C 驱动 — 全缓冲 + 硬件 I2C（优化版） */
#include "oled.h"
#include "font5x7.h"

/* ===================== 硬件常量 ===================== */
#define SSD1306_ADDR  0x3C  /* 7-bit I2C 地址，SA0=0 */

/* 显存：128×64 / 8 = 1024 字节，8 页 × 128 列 */
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES    8

static uint8_t vram[OLED_WIDTH * OLED_PAGES];

/* I2C 句柄（外部传入） */
static I2C_HandleTypeDef *oled_i2c;

/* 光标 */
static uint8_t cursor_x, cursor_y;

/* ===================== I2C 发送底层 ===================== */

/* 发送命令字节：MemAddress=0x00 表示 D/C#=0（命令模式） */
static void write_cmd(uint8_t cmd)
{
    HAL_I2C_Mem_Write(oled_i2c, SSD1306_ADDR << 1,
                      0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
}

/* 批量发送数据字节：MemAddress=0x40 表示 D/C#=1（数据模式），单次事务 */
static void write_data(uint8_t *data, uint16_t len)
{
    HAL_I2C_Mem_Write(oled_i2c, SSD1306_ADDR << 1,
                      0x40, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/* ===================== SSD1306 初始化序列 ===================== */
void oled_init(I2C_HandleTypeDef *hi2c)
{
    oled_i2c = hi2c;

    write_cmd(0xAE);  /* Display OFF */
    write_cmd(0xD5);  /* Set OSC Freq */
    write_cmd(0x80);
    write_cmd(0xA8);  /* Set MUX Ratio */
    write_cmd(0x3F);  /* 64 lines */
    write_cmd(0xD3);  /* Set Display Offset */
    write_cmd(0x00);
    write_cmd(0x40);  /* Start Line = 0 */
    write_cmd(0x8D);  /* Charge Pump */
    write_cmd(0x14);  /* Enable */
    write_cmd(0x20);  /* Memory Mode */
    write_cmd(0x00);  /* Horizontal */
    write_cmd(0xA1);  /* Segment Remap (左右镜像) */
    write_cmd(0xC8);  /* COM Scan Direction (上下镜像) */
    write_cmd(0xDA);  /* COM Pins */
    write_cmd(0x12);
    write_cmd(0x81);  /* Contrast */
    write_cmd(0xCF);
    write_cmd(0xD9);  /* Pre-charge Period */
    write_cmd(0xF1);
    write_cmd(0xDB);  /* VCOMH Deselect */
    write_cmd(0x40);
    write_cmd(0xA4);  /* Display from RAM */
    write_cmd(0xA6);  /* Normal display (不反白) */
    write_cmd(0xAF);  /* Display ON */

    oled_clear();
    oled_show();
}

/* ===================== 显存操作 ===================== */

void oled_clear(void)
{
    for (uint16_t i = 0; i < sizeof(vram); i++) vram[i] = 0;
}

void oled_show(void)
{
    /*
     * 事务 1：合并 6 条命令 → 1 次 I2C 事务
     * 0x21 = 设列地址, 0x22 = 设页地址
     * Co=0 时每条命令结束后下一字节自动视为新控制字节，
     * 所以 0x22 在 127 之后会被识别为"下一条命令"而非数据。
     */
    uint8_t cmds[] = {0x21, 0, 127, 0x22, 0, 7};
    HAL_I2C_Mem_Write(oled_i2c, SSD1306_ADDR << 1,
                      0x00, I2C_MEMADD_SIZE_8BIT, cmds, sizeof(cmds), 10);

    /* 事务 2：1024 字节显存一次写入（不再分包） */
    HAL_I2C_Mem_Write(oled_i2c, SSD1306_ADDR << 1,
                      0x40, I2C_MEMADD_SIZE_8BIT, vram, sizeof(vram), 100);
}

/* 画一个像素（仅在显存中，不刷新） */
static void draw_pixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint8_t page = y >> 3;
    uint8_t bit  = y & 0x07;
    if (color)
        vram[page * OLED_WIDTH + x] |=  (1 << bit);
    else
        vram[page * OLED_WIDTH + x] &= ~(1 << bit);
}

/* 在 (x, y) 处画 5×7 字符（y 以像素为单位） */
static void draw_char(uint8_t x, uint8_t y, char c)
{
    if (c < 0x20 || c > 0x7E) c = ' ';
    const uint8_t *glyph = font5x7[c - 0x20];

    for (uint8_t col = 0; col < 5; col++) {
        for (uint8_t row = 0; row < 7; row++) {
            draw_pixel(x + col, y + row, (glyph[col] >> row) & 1);
        }
    }
}

/* ===================== 文本绘制 ===================== */

void oled_set_cursor(uint8_t x, uint8_t y)
{
    cursor_x = x;
    cursor_y = y;
}

void oled_print(const char *str)
{
    uint8_t x = cursor_x;
    uint8_t y = cursor_y;

    while (*str) {
        if (x + 5 >= OLED_WIDTH) {
            x = 0;
            y++;
        }
        if (y >= OLED_PAGES) break;

        draw_char(x, y * 8, *str);
        x += 6;
        str++;
    }
    cursor_x = x;
    cursor_y = y;
}

/* ===================== 快捷方法 ===================== */

void oled_print_uint(uint32_t v)
{
    char buf[11];                          /* 最大 10 位 + \0 */
    uint8_t i = sizeof(buf) - 1;
    buf[i--] = '\0';
    if (v == 0) buf[i--] = '0';
    else while (v) { buf[i--] = '0' + (v % 10); v /= 10; }
    oled_print(&buf[i + 1]);
}

void oled_show_string(uint8_t line, const char *str)
{
    oled_clear();
    oled_set_cursor(0, line);
    oled_print(str);
    oled_show();
}
