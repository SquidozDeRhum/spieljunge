#include <iostream>

#include <raylib.h>

#include "../include/rendering.hpp"
#include "../include/const.hpp"

std::vector<uint16_t> get_tile(uint16_t ID, std::vector<uint8_t>& RAM) {
    std::vector<uint16_t> tile;

    for (int i = VRAM_START + (ID * 16); i < VRAM_START + (ID * 16) + 16; i += 2) {
        uint16_t lowByte = RAM[i];
        uint16_t highByte = RAM[i + 1];

        // To combine the two bytes, we need to do bit interleaving
        lowByte = (lowByte | (lowByte << 4)) & 0x0F0F;
        lowByte = (lowByte | (lowByte << 2)) & 0x3333;
        lowByte = (lowByte | (lowByte << 1)) & 0x5555;

        highByte = (highByte | (highByte << 4)) & 0x0F0F;
        highByte = (highByte | (highByte << 2)) & 0x3333;
        highByte = (highByte | (highByte << 1)) & 0x5555;

        uint16_t line = lowByte | (highByte << 1);

        tile.push_back(line);
    }

    return tile;
}

void draw_line(std::vector<uint8_t>& RAM, Image& screen) {
    // We first need to know which line of the tile map we need

    int ptd = 0; // pixel to draw

    for (int i = 0; i < 32; i++) {

        int jaaj = VRAM_FIRST_TL + (RAM[SCY] / 8) * 32 + RAM[SCX] + i + (RAM[LY] / 8) * 32;

        uint16_t tl = get_tile(RAM[jaaj], RAM)[RAM[LY] % 8]; // tile line

        for (int j = 14; j >= 0; j -= 2) {
            switch ((tl >> j) & 0x3)
            {
            case FIRST_COLOR:
                ImageDrawRectangle(&screen, ptd, RAM[LY] * 3, 3, 3, BLANK);
                break;
            case SECOND_COLOR:
                ImageDrawRectangle(&screen, ptd, RAM[LY] * 3, 3, 3, LIGHTGRAY);
                break;
            case THIRD_COLOR:
                ImageDrawRectangle(&screen, ptd, RAM[LY] * 3, 3, 3, GRAY);
                break;
            case FOURTH_COLOR:
                ImageDrawRectangle(&screen, ptd, RAM[LY] * 3, 3, 3, BLACK);
                break;
            default:
                std::cout << "WTF" << std::endl;
                break;
            }
            
            ptd += 3;
        }
    }
}