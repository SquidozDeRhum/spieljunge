#include <iostream>
#include <fstream>
#include <iomanip>
#include <bitset>
#include <vector>
#include <string>
#include <cstdint>

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

    std::vector<uint8_t> RAM = std::vector<uint8_t>(0xFFFF, 0x00);

    loadROM(RAM, ROMPATH);

    // displayROM(RAM);

    while (true) {
        std::cout << "PC : 0x" << std::hex << PC << std::endl;
        std::cout << "Instruction : 0x" << std::hex << +RAM[PC] << std::endl;
        std::cout << "A : 0x" << std::hex << +A << std::endl;

        ECI(A, F, B, C, D, E, H, L, SP, PC, RAM);
    }

    return EXIT_SUCCESS;
}