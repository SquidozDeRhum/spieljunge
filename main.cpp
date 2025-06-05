#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <bitset>
#include <cstdint>

#include "./include/tools.hpp"
#include "./include/const.hpp"

int main() {

    u_int8_t A;
    u_int8_t F; // used for Z N H C flags
    u_int8_t B;
    u_int8_t C;
    u_int8_t D;
    u_int8_t E;
    u_int8_t H;
    u_int8_t L;

    u_int16_t SP;
    u_int16_t PC;

    std::vector<u_int8_t> RAM = std::vector<u_int8_t>(0xFFFF, 0x00);

    loadROM(RAM, ROMPATH);

    // displayROM(RAM);

    uint8_t test = 0xFF;

    int8_t adjust = -0x66;

    std::cout << ~test << std::endl;

    return EXIT_SUCCESS;
}