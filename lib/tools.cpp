#include "../include/tools.hpp"
#include "../include/const.hpp"

void loadROM(std::vector<u_int8_t>& RAM, std::string filename) {
    char value;

    std::fstream rom;
    rom.open(filename, std::fstream::in | std::fstream::binary);

    u_int16_t counter = 0x0000;

    while (rom.read(&value, 1) && counter < ROMSIZE) {
        RAM[counter] = value;
        counter++;
    }
}

void displayROM(std::vector<u_int8_t>& RAM) {
    for (int i = 1; i < ROMSIZE + 2; i++) {
        std::cout << std::hex << int(RAM[i - 1]) << "\t";
        if (i % 0x10 == 0) {
            std::cout << std::endl;
        }
    }

    std::cout << std::endl;
}