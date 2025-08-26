#include <iostream>
#include <fstream>
#include <iomanip>
#include <bitset>
#include <vector>
#include <string>
#include <cstdint>
#include <chrono>

#include <raylib.h>

#include "./include/tools.hpp"
#include "./include/const.hpp"

#include "./include/instruction.hpp"
#include "./include/prefixed.hpp"

#include "./include/rendering.hpp"

int main() {

    uint8_t A = 0x01;
    uint8_t F = 0xB0; // used for Z N H C flags
    uint8_t B = 0x00;
    uint8_t C = 0x13;
    uint8_t D = 0x00;
    uint8_t E = 0xD8;
    uint8_t H = 0x01;
    uint8_t L = 0x4D;

    uint16_t SP = SP_START;
    uint16_t PC = 0x0000;

    int cycles_counter = 0;
    
    std::vector<uint8_t> RAM = std::vector<uint8_t>(0xFFFF, 0x00);
    
    loadROM(RAM, ROMPATH);

    loadBoot(RAM, BOOTPATH);

    InitWindow(680, 432, "Spieljunge");

    int breakpoint = 0x5b;
    bool breakpoint_reached = false;

    Image screen = GenImageColor(480, 432, BLACK);
    Texture screen_tex = LoadTextureFromImage(screen);

    bool screen_updated = false;

    while (!WindowShouldClose()) {
        if (IsKeyDown(KEY_N)) {

            bool loop_done = false;

            if (RAM[LY] < 144) {
                loop_done = true;
            }

            bool zero_passed = false;
            
            while (RAM[LY] != 144 || !loop_done) {
                ECI(A, F, B, C, D, E, H, L, SP, PC, RAM, cycles_counter);
                
                if (cycles_counter >= 114) {
                    if ((RAM[LCDC] & 0x80) != 0) {
                        draw_line(RAM, screen);
                        RAM[LY] += 1;
                    }
                    cycles_counter -= 114;
                }

                if (RAM[LY] == 154) {
                    RAM[LY] = 0;
                    screen_updated = false;
                    zero_passed = true;
                }

                if (RAM[LY] == 144 && zero_passed) {
                    loop_done = true;
                }
            }

        } else if (IsKeyDown(KEY_SPACE) || IsKeyPressed(KEY_S) || !breakpoint_reached) {
            ECI(A, F, B, C, D, E, H, L, SP, PC, RAM, cycles_counter);

            if (cycles_counter >= 114) {
                if ((RAM[LCDC] & 0x80) != 0) {
                    draw_line(RAM, screen);
                    RAM[LY] += 1;
                }
                cycles_counter -= 114;
            }

            if (RAM[LY] == 154) {
                RAM[LY] = 0;
                screen_updated = false;
            }
        }
        
        if (PC == breakpoint) {
            breakpoint_reached = true;
        }
        
        if (breakpoint_reached) {
            BeginDrawing();
            
                DrawRectangle(0, 0, 200, 432, WHITE);
                DrawText(("PC : " + R16_to_str(PC)).c_str(), 10, 10, 20, BLACK);
                DrawText(("AF : " + R16_to_str((A << 8) | F)).c_str(), 10, 35, 20, BLACK);
                DrawText(("BC : " + R16_to_str((B << 8) | C)).c_str(), 10, 60, 20, BLACK);
                DrawText(("DE : " + R16_to_str((D << 8) | E)).c_str(), 10, 85, 20, BLACK);
                DrawText(("HL : " + R16_to_str((H << 8) | L)).c_str(), 10, 110, 20, BLACK);
                DrawText(("LY : " + R8_to_str(RAM[LY])).c_str(), 10, 150, 20, RED);
                DrawText(("M-Cycles : " + std::to_string(cycles_counter)).c_str(), 10, 175, 20, PURPLE);
                DrawText(("SP : " + R16_to_str(SP)).c_str(), 10, 205, 20, GREEN);
                DrawText(("Inst : " + R16_to_str(RAM[PC])).c_str(), 10, 230, 20, RED);
                
                DrawFPS(10, 255);
                

                if (RAM[LY] == 144 && !screen_updated) {

                    UpdateTexture(screen_tex, screen.data);
                    DrawRectangle(SCREEN_X_START, 0, 480, 432, BLACK);
                    DrawTexture(screen_tex, SCREEN_X_START, 0, WHITE);

                    screen_updated = true;
                }
        
            EndDrawing();

        }

    }

    CloseWindow();

    return EXIT_SUCCESS;
}