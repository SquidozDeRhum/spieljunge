#include <iostream>

#include <raylib.h>

#include "../include/rendering.hpp"
#include "../include/const.hpp"
#include "../include/types.hpp"

uint16_t getTileLine(uint8_t ID, uint8_t lineNumber, bool object, std::vector<uint8_t>& RAM) {
    uint16_t selectedBlockAddress;

    // get the address of the line of the tile we want depending on addressing mode
    if ((RAM[LCDC] & 0x10) != 0 || object) {
        selectedBlockAddress = BLOCK0_START + ID * 16 + lineNumber * 2;
    } else {
        selectedBlockAddress = BLOCK2_START + (int8_t)ID * 16 + lineNumber * 2;
    }    

    uint16_t lowByte = RAM[selectedBlockAddress];
    uint16_t highByte = RAM[selectedBlockAddress + 1];

    // To combine the two bytes, we need to do bit interleaving
    lowByte = (lowByte | (lowByte << 4)) & 0x0F0F;
    lowByte = (lowByte | (lowByte << 2)) & 0x3333;
    lowByte = (lowByte | (lowByte << 1)) & 0x5555;

    highByte = (highByte | (highByte << 4)) & 0x0F0F;
    highByte = (highByte | (highByte << 2)) & 0x3333;
    highByte = (highByte | (highByte << 1)) & 0x5555;

    uint16_t line = lowByte | (highByte << 1);

    return line;
}

void draw_line(std::vector<uint8_t>& RAM, Image& screen) {
    Color colors[4] = {BLANK, LIGHTGRAY, GRAY, BLACK};

    // LCDC.3 control object height, 0 = 8x8 and 1 = 8x16
    uint8_t tileHeight = (RAM[LCDC] & 0x04) == 0 ? 8 : 16;

    // We get all the objects on the current scanline
    std::vector<Object> objects;
    for (int i = OAM_START; i <= OAM_END - 3; i += 4) {
        uint8_t objectY = RAM[i] - 16;

        if (objectY >= RAM[LY] - tileHeight + 1 && objectY <= RAM[LY]) {
            objects.push_back(Object{
                RAM[i + 1], 
                RAM[i + 3],
                getTileLine(RAM[i + 2], RAM[LY] - objectY, true, RAM)
            });
        }
    }

    int ptd = 0; // pixel to draw
    for (int i = 0; i < 32; i++) {

        uint16_t offset = (RAM[SCY] / 8) * 32 + RAM[SCX] + i + (RAM[LY] / 8) * 32;

        // Get the background tile line
        // Please never touch to this formula it took me a day to figure it out 
        uint16_t tileAddress = ((RAM[LCDC] & 0x08) == 0 ? VRAM_FIRST_TM_START :  VRAM_SECOND_TM_START) + offset;
        uint16_t backgroundTL = getTileLine(RAM[tileAddress], RAM[LY] % 8, false, RAM);

        // Get the window tile line
        tileAddress = ((RAM[LCDC] & 0x40) == 0 ? VRAM_FIRST_TM_START : VRAM_SECOND_TM_START) + offset;
        uint16_t windowTL = getTileLine(RAM[tileAddress], RAM[LY] % 8, false, RAM);

        uint8_t colorToDraw;

        for (int j = 14; j >= 0; j -= 2) {

            // LCDC.0 -> BG/Window enable
            if ((RAM[LCDC] & 0x01) != 0) {
                if (((windowTL >> j) & 0x03) != FIRST_COLOR && (RAM[LCDC] & 0x20) != 0) {
                    colorToDraw = (windowTL >> j) & 0x03;
                } else {
                    colorToDraw = (backgroundTL >> j) & 0x03;
                }
            } else {
                colorToDraw = FIRST_COLOR;
            }

            for (int k = objects.size() - 1; k >= 0; k--) {
                uint8_t objectX = objects[k].x - 8;

                if (objectX >= ptd - SPRITE_WIDTH + 1 && objectX <= ptd) {
                    colorToDraw = (objects[k].tileLine >> (SPRITE_WIDTH - 1 - (ptd - objectX)) * 2) & 0x03;
                }
            }

            ImageDrawRectangle(&screen, ptd * PIXEL_SIZE, RAM[LY] * PIXEL_SIZE, PIXEL_SIZE, PIXEL_SIZE, colors[colorToDraw]);
            
            ptd++;
        }
    }
}