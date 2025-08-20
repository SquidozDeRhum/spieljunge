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

    int breakpoint = 0xfe;

    while (!WindowShouldClose()) {
        if (IsKeyDown(KEY_SPACE) || IsKeyPressed(KEY_S) || PC < breakpoint) {

            if (cycles_counter >= 114 && (RAM[LCDC] & 0x80) != 0) {
                RAM[LY] += 1;
                if (RAM[LY] == 154) {
                    RAM[LY] = 0;
                }
                cycles_counter -= 114;
            }
            
            ECI(A, F, B, C, D, E, H, L, SP, PC, RAM, cycles_counter);

        }

        if (PC >= breakpoint) {
            BeginDrawing();
        
                    ClearBackground(RAYWHITE);
                    DrawText(("PC : " + R16_to_str(PC)).c_str(), 10, 10, 20, BLACK);
                    DrawText(("AF : " + R16_to_str((A << 8) | F)).c_str(), 10, 35, 20, BLACK);
                    DrawText(("BC : " + R16_to_str((B << 8) | C)).c_str(), 10, 60, 20, BLACK);
                    DrawText(("DE : " + R16_to_str((D << 8) | E)).c_str(), 10, 85, 20, BLACK);
                    DrawText(("HL : " + R16_to_str((H << 8) | L)).c_str(), 10, 110, 20, BLACK);
                    DrawText(("LY : " + R8_to_str(RAM[LY])).c_str(), 10, 150, 20, RED);
                    DrawText(("M-Cycles : " + std::to_string(cycles_counter)).c_str(), 10, 175, 20, PURPLE);
                    DrawText(("SP : " + R16_to_str(SP)).c_str(), 10, 205, 20, GREEN);
                    DrawText(("Instruction : " + R16_to_str(RAM[PC])).c_str(), 10, 230, 20, RED);
    
        
            EndDrawing();
        }

    }

    CloseWindow();

    std::cout << "LCDC : " << +RAM[LCDC] << std::endl;
    std::cout << "9910 : " << +RAM[0x990F] << std::endl;

    displayMemorySection(RAM, 0x8000, 0x9FFF);

    /*


    RAM[LCDC] = 0x91;

    displayROM(RAM);
    
    InitWindow(500, 500, "Spieljunge");

    bool reached = false;

    auto start = std::chrono::high_resolution_clock::now();

    while (!WindowShouldClose()) {

        if (IsKeyDown(KEY_SPACE) || IsKeyPressed(KEY_S) || PC < breakpoint) {

            if (cycles_counter >= 114 && (RAM[LCDC] & 0x80) != 0) {
                RAM[LY] += 1;
                if (RAM[LY] == 154) {
                    RAM[LY] = 0;
                }
                cycles_counter -= 114;
            }
            
            ECI(A, F, B, C, D, E, H, L, SP, PC, RAM, cycles_counter);

        }

        if (PC == breakpoint && !reached) {
            reached = true;

            auto stop = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

            std::cout << "Time to reach breakpoint : " << duration.count() << " microseconds." << std::endl;
        }

        if (PC >= breakpoint) {     
            BeginDrawing();
    
                ClearBackground(RAYWHITE);
                DrawText(("PC : " + R16_to_str(PC)).c_str(), 10, 10, 20, BLACK);
                DrawText(("SP : " + R16_to_str(SP)).c_str(), 200, 10, 20, GREEN);
                DrawText(("AF : " + R16_to_str((A << 8) | F)).c_str(), 10, 35, 20, BLACK);
                DrawText(("BC : " + R16_to_str((B << 8) | C)).c_str(), 10, 60, 20, BLACK);
                DrawText(("DE : " + R16_to_str((D << 8) | E)).c_str(), 10, 85, 20, BLACK);
                DrawText(("HL : " + R16_to_str((H << 8) | L)).c_str(), 10, 110, 20, BLACK);
                DrawText(("LY : " + R8_to_str(RAM[LY])).c_str(), 10, 150, 20, RED);

                DrawText(("M-Cycles : " + std::to_string(cycles_counter)).c_str(), 10, 175, 20, PURPLE);
    
            EndDrawing();
        }

    }

    CloseWindow();

    */

    return EXIT_SUCCESS;
}