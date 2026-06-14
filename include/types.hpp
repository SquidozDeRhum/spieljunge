#include <cstdint>

#include "./const.hpp"

struct Registers {
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

    bool preIME = false;
    bool IME = false;
    bool VBLANKTriggered = false;

    int cycles_counter = 0;
};

struct Object {
    uint8_t x;
    uint8_t AF;
    uint16_t tileLine;
};