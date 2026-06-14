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

    Registers registers;
    
    std::vector<uint8_t> RAM = std::vector<uint8_t>(0x10000, 0x00);

    // Initialization of RAM values
    RAM[JOYP] = 0xCF;
    
    loadROM(RAM, ROMPATH);
    loadBoot(RAM, BOOTPATH);

    bool DMGEnabled = true;
    
    InitWindow(680, 432, "Spieljunge");
    SetTargetFPS(60);

    int breakpoint = 0x00; // 0xF7
    bool breakpoint_reached = false;

    Image screen = GenImageColor(480, 432, BLACK);
    Texture screen_tex = LoadTextureFromImage(screen);

    while (!WindowShouldClose()) {
        if (registers.PC == breakpoint) {
            breakpoint_reached = true;
        }
        
        if (RAM[BOOTROM] & 0x01 == 1 && DMGEnabled) {
            loadROM(RAM, ROMPATH);
            DMGEnabled = false;
        }
        
        if (IsKeyDown(KEY_N) || IsKeyPressed(KEY_F)) {
            
            bool loop_done = false;
            
            if (RAM[LY] < 144) {
                loop_done = true;
            }
            
            bool zero_passed = false;
            
            while (RAM[LY] != 144 || !loop_done) {

                doCPUStuff(registers, RAM, screen);
                
                if (RAM[LY] == 0) {
                    zero_passed = true;
                }
                
                if (RAM[LY] == 144 && zero_passed) {
                    loop_done = true;
                }
            }
            
        } else if (IsKeyDown(KEY_SPACE) || IsKeyPressed(KEY_I) || !breakpoint_reached) {
            doCPUStuff(registers, RAM, screen);
        }
        
        if (breakpoint_reached) {
            BeginDrawing();

                ClearBackground(WHITE);
            
                DrawText(("PC : " + R16_to_str(registers.PC)).c_str(), 10, 10, 20, BLACK);
                DrawText(("AF : " + R16_to_str((registers.A << 8) | registers.F)).c_str(), 10, 35, 20, BLACK);
                DrawText(("BC : " + R16_to_str((registers.B << 8) | registers.C)).c_str(), 10, 60, 20, BLACK);
                DrawText(("DE : " + R16_to_str((registers.D << 8) | registers.E)).c_str(), 10, 85, 20, BLACK);
                DrawText(("HL : " + R16_to_str((registers.H << 8) | registers.L)).c_str(), 10, 110, 20, BLACK);
                DrawText(("LY : " + R8_to_str(RAM[LY])).c_str(), 10, 150, 20, RED);
                DrawText(("M-Cycles : " + std::to_string(registers.cycles_counter)).c_str(), 10, 175, 20, PURPLE);
                DrawText(("SP : " + R16_to_str(registers.SP)).c_str(), 10, 205, 20, GREEN);
                DrawText(("Inst : " + R16_to_str(RAM[registers.PC])).c_str(), 10, 230, 20, RED);
                
                DrawFPS(10, 255);

                UpdateTexture(screen_tex, screen.data);
                DrawTexture(screen_tex, SCREEN_X_START, 0, WHITE);
        
            EndDrawing();
        }
    }

    CloseWindow();

    return EXIT_SUCCESS;
}