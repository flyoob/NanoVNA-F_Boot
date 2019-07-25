/*-----------------------------------------------------------------------------/
 * Module       : nt35510.h
 * Create       : 2019-05-23
 * Copyright    : hamelec.taobao.com
 * Author       : huanglong
 * Brief        : NT35510
 TK043F1508, The is a 480(RGB)x800 dot-matrix TFT module.
/-----------------------------------------------------------------------------*/
#ifndef _NT35510_H
#define _NT35510_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define BRG556(b,r,g)     ( (((r)<<8)&0xf800) | (((g)<<3)&0x07e0) | (((b)>>3)&0x001f) )

#define   BLACK                0x0000                // 黑色：    0,   0,   0 //
// #define   BLUE                 0x001F                // 蓝色：    0,   0, 255 //
// #define   GREEN                0x07E0                // 绿色：    0, 255,   0 //
// #define   CYAN                 0x07FF                // 青色：    0, 255, 255 //
// #define   RED                  0xF800                // 红色：  255,   0,   0 //
// #define   MAGENTA              0xF81F                // 品红：  255,   0, 255 //
// #define   YELLOW               0xFFE0                // 黄色：  255, 255, 0   //
#define   WHITE                0xFFFF                // 白色：  255, 255, 255 //
// #define   NAVY                 0x000F                // 深蓝色：  0,   0, 128 //
// #define   DGREEN               0x03E0                // 深绿色：  0, 128,   0 //
// #define   DCYAN                0x03EF                // 深青色：  0, 128, 128 //
// #define   MAROON               0x7800                // 深红色：128,   0,   0 //
// #define   PURPLE               0x780F                // 紫色：  128,   0, 128 //
// #define   OLIVE                0x7BE0                // 橄榄绿：128, 128,   0 //
// #define   LGRAY                0xC618                // 灰白色：192, 192, 192 //
// #define   DGRAY                0x7BEF                // 深灰色：128, 128, 128 //

#define COLOR_INFO   BRG556(0, 247,  80)
#define COLOR_BURN   BRG556(0, 140, 234)

#define lcd_debug(format, ...) { \
  sprintf(chr_buffer, format, ##__VA_ARGS__); \
  nt35510_debug(chr_buffer, COLOR_INFO, BLACK); \
}

extern char     chr_buffer[1024];
extern uint16_t lcd_buffer[4096];

void nt35510_fill(int x, int y, int w, int h, int color);
void nt35510_init(void);
void nt35510_debug(char *str, uint16_t fg, uint16_t bg);
void nt35510_burn(char *str, uint16_t fg, uint16_t bg);

#endif
