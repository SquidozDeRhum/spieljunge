#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <bitset>
#include <cstdint>

#include "./include/tools.hpp"
#include "./include/const.hpp"

#include "./include/opcodeR.hpp"

int main() {

    uint8_t A;
    uint8_t F; // used for Z N H C flags
    uint8_t B;
    uint8_t C;
    uint8_t D;
    uint8_t E;
    uint8_t H;
    uint8_t L;

    uint16_t SP;
    uint16_t PC;

    std::vector<uint8_t> RAM = std::vector<uint8_t>(0xFFFF, 0x00);

    loadROM(RAM, ROMPATH);

    // displayROM(RAM);

    A = 0x10;
    PC = 0;
    F = 0b00010000;

    DAA(PC, A, F);

    std::cout << std::hex << std::uppercase << (int)A << std::endl;

    flagsOutput(F);

    return EXIT_SUCCESS;
}