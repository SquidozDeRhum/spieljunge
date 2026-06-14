#pragma once

#define ROMSIZE 0x7FFF
#define ROMPATH "./assets/tetris.gb"

#define BOOTSIZE 0xFF
#define BOOTPATH "./assets/dmg_boot.bin"

#define ZERO_FLAG 0b10000000
#define NEGATIVE_FLAG 0b01000000
#define HALF_CARRY_FLAG 0b00100000
#define CARRY_FLAG 0b00010000

#define NO_FLAG 0x00

// Don't know how to name this category

#define SPRITE_WIDTH 8

// Useful adresses

#define BLOCK0_START 0x8000
#define BLOCK1_START 0x8800
#define BLOCK2_START 0x9000

#define VRAM_END 0x9FFF
#define VRAM_FIRST_TM_START 0x9800
#define VRAM_SECOND_TM_START 0x9C00

#define OAM_START 0xFE00
#define OAM_END 0xFE9F


// Registers

#define JOYP 0xFF00

#define LY 0xFF44
#define SCY 0xFF42
#define SCX 0xFF43
#define LCDC 0xFF40

#define BOOTROM 0xFF50

#define IF 0xFF0F

#define DMA 0xFF46

#define SP_START 0xFFFE

#define IE 0xFFFF

// Interrupts

#define VBLANK 0x01
#define LCD 0x02
#define TIMER 0x04
#define SERIAL 0x08
#define JOYPAD 0x10

#define VBLANK_VECTOR 0x0040
#define LCD_VECTOR 0x0048
#define TIMER_VECTOR 0x0050
#define SERIAL_VECTOR 0x0058
#define JOYPAD_VECTOR 0x0060

// Spieljunge

#define NB_DAA_CHECK 2

#define SCREEN_X_START 200

#define FIRST_COLOR 0x00
#define SECOND_COLOR 0x01
#define THIRD_COLOR 0x02
#define FOURTH_COLOR 0x03

#define PIXEL_SIZE 3

const char hex[0x10] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

const int instructions_cycles[0x100] = 
{
1, 3, 2, 2, 1, 1, 2, 1, 5, 2, 2, 2, 1, 1, 2, 1,
1, 3, 2, 2, 1, 1, 2, 1, 3, 2, 2, 2, 1, 1, 2, 1,
2, 3, 2, 2, 1, 1, 2, 1, 2, 2, 2, 2, 1, 1, 2, 1,
2, 3, 2, 2, 3, 3, 3, 1, 2, 2, 2, 2, 1, 1, 2, 1,
1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
2, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 2, 1,
1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
2, 3, 3, 4, 3, 4, 2, 4, 2, 4, 3, 1, 3, 6, 2, 4,
2, 3, 3, 0, 3, 4, 2, 4, 2, 4, 3, 0, 3, 0, 2, 4,
3, 3, 2, 0, 0, 4, 2, 4, 4, 1, 4, 0, 0, 0, 2, 4,
3, 3, 2, 1, 0, 4, 2, 4, 3, 2, 4, 1, 0, 0, 2, 4
};

const int prefixed_cycles[0x100] = {
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2,
2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2,
2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2,
2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 3, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
};