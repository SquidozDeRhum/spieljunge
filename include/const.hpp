#pragma once

#define ROMSIZE 0x7FFF
#define ROMPATH "./src/tetris.gb"

#define ZERO_FLAG 0b10000000
#define NEGATIVE_FLAG 0b01000000
#define HALF_CARRY_FLAG 0b00100000
#define CARRY_FLAG 0b00010000

#define SP_START 0xFFFE

#define NB_DAA_CHECK 2