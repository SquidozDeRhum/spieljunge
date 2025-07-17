#include <iostream>
#include <fstream>
#include <iomanip>
#include <bitset>
#include <vector>
#include <string>
#include <cstdint>

#include "./include/tools.hpp"
#include "./include/const.hpp"

#include "./include/opcode.hpp"
#include "./include/prefixed.hpp"

int main() {

    uint8_t A;
    uint8_t F = 0x00; // used for Z N H C flags
    uint8_t B;
    uint8_t C;
    uint8_t D;
    uint8_t E;
    uint8_t H;
    uint8_t L;

    uint16_t SP = SP_START;
    uint16_t PC = 0;

    std::vector<uint8_t> RAM = std::vector<uint8_t>(0xFFFF, 0x00);

    loadROM(RAM, ROMPATH);

    // displayROM(RAM);

    uint8_t test = 0b11111111;

    RAM[0x3458] = test;

    RES_N_ADHL(PC, RAM, 0x34, 0x58, 3);

    std::cout <<  std::bitset<8>(RAM[0xE780]) << std::endl;

    return EXIT_SUCCESS;
}