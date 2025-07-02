#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
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

    uint16_t SP = SP_START;
    uint16_t PC;

    std::vector<uint8_t> RAM = std::vector<uint8_t>(0xFFFF, 0x00);

    loadROM(RAM, ROMPATH);

    // displayROM(RAM);

    uint16_t test = 0xF;

    int8_t test2 = 0x1;

    std::cout << std::hex << test + test2 << std::endl;

    return EXIT_SUCCESS;
}