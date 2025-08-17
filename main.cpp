#include <iostream>
#include <fstream>
#include <iomanip>
#include <bitset>
#include <vector>
#include <string>
#include <cstdint>

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
    uint16_t PC = 0x0100;

    int cycles_counter = 0;

    std::vector<uint8_t> RAM = std::vector<uint8_t>(0xFFFF, 0x00);

    loadROM(RAM, ROMPATH);

    // displayROM(RAM);

    InitWindow(500, 500, "Spieljunge");

    while (!WindowShouldClose()) {

        if (IsKeyPressed(KEY_SPACE)) {
            
            if (cycles_counter > 456) {
                RAM[LY] += 1;
                if (RAM[LY] == 154) {
                    RAM[LY] = 0;
                }
                cycles_counter -= 456;
            }
            
            ECI(A, F, B, C, D, E, H, L, SP, PC, RAM, cycles_counter);

        }


        BeginDrawing();

            ClearBackground(RAYWHITE);
            DrawText(("PC : " + R16_to_str(PC)).c_str(), 10, 10, 20, BLACK);
            DrawText(("B : " + R8_to_str(B)).c_str(), 10, 35, 20, BLACK);
            DrawText(("C : " + R8_to_str(C)).c_str(), 10, 60, 20, BLACK);

        EndDrawing();

    }

    CloseWindow();

    return EXIT_SUCCESS;
}