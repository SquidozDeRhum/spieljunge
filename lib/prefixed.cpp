#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdint>

#include "../include/opcode.hpp"
#include "../include/const.hpp"
#include "../include/tools.hpp"

void RLC_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    uint8_t oldR = R;

    R = (R << 1);

    if ((oldR & 0b10000000) != 0) {
        F |= CARRY_FLAG; // set carry flag
        R |= 0b00000001;
    }

    if (R == 0) {
        F |= ZERO_FLAG;
    }

    PC ++;
}

void RLC_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= NO_FLAG;

    uint16_t HL = (H << 8) | L;
    uint8_t oldValue = RAM[HL];

    RAM[HL] = (RAM[HL] << 1);

    if ((oldValue & 0b10000000) != 0) {
        F |= CARRY_FLAG;
        RAM[HL] |= 0b0000001;
    }

    if (RAM[HL] == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void RRC_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    uint8_t oldR = R;

    R = (R >> 1);

    if ((oldR & 0b00000001) != 0) {
        F |= CARRY_FLAG; // set carry flag
        R |= 0b10000000;
    }

    if (R == 0) {
        F |= ZERO_FLAG;
    }

    PC ++;
}

void RRC_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= NO_FLAG;

    uint16_t HL = (H << 8) | L;
    uint8_t oldValue = RAM[HL];

    RAM[HL] = (RAM[HL] >> 1);

    if ((oldValue & 0b00000001) != 0) {
        F |= CARRY_FLAG;
        RAM[HL] |= 0b10000000;
    }

    if (RAM[HL] == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void RL_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    uint8_t oldR = R;

    R = (R << 1);

    if ((F & CARRY_FLAG) != 0) {
        R |= 0b00000001;
    }

    if (R == 0) {
        F |= ZERO_FLAG;
    }

    F = NO_FLAG;

    if ((oldR & 0b10000000) != 0) {
        F |= CARRY_FLAG;
    }

    PC++;
}

void RL_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    uint16_t HL = (H << 8) | L;
    uint8_t oldValue = RAM[HL];

    RAM[HL] = (RAM[HL] << 1);

    if ((F & CARRY_FLAG) != 0) {
        RAM[HL] |= 0b00000001;
    }

    if (RAM[HL] == 0) {
        F |= ZERO_FLAG;
    }

    F = NO_FLAG;

    if ((oldValue & 0b10000000)) {
        F |= CARRY_FLAG;
    }

    PC++;
}

void RR_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    uint8_t oldR = R;

    R = (R >> 1);

    if ((F & CARRY_FLAG) != 0) {
        R |= 0b10000000;
    }

    if (R == 0) {
        F |= ZERO_FLAG;
    }

    F = NO_FLAG;

    if ((oldR & 0b00000001) != 0) {
        F |= CARRY_FLAG;
    }

    PC++;
}

void RR_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    uint16_t HL = (H << 8) | L;
    uint8_t oldValue = RAM[HL];

    RAM[HL] = (RAM[HL] >> 1);

    if ((F & CARRY_FLAG) != 0) {
        RAM[HL] |= 0b10000000;
    }

    if (RAM[HL] == 0) {
        F |= ZERO_FLAG;
    }

    F = NO_FLAG;

    if ((oldValue & 0b00000001)) {
        F |= CARRY_FLAG;
    }

    PC++;
}

void SLA_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F = NO_FLAG;

    if ((R & 0b10000000) != 0) {
        F |= CARRY_FLAG;
    }

    R  = (R << 1);

    if (R == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SLA_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F = NO_FLAG;

    uint16_t HL = (H << 8) | L;

    if ((RAM[HL] & 0b10000000) != 0) {
        F |= CARRY_FLAG;
    }

    RAM[HL] = (RAM[HL] << 1);

    if (RAM[HL] == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SRA_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F = NO_FLAG;

    uint8_t oldR = R;

    if ((R & 0b00000001) != 0) {
        F |= CARRY_FLAG;
    }

    R  = (R >> 1);

    if ((oldR & 0b10000000) != 0) {
        R |= 0b10000000;
    }

    if (R == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SRA_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F = NO_FLAG;

    uint16_t HL = (H << 8) | L;

    uint8_t oldValue = RAM[HL];

    if ((RAM[HL] & 0b00000001) != 0) {
        F |= CARRY_FLAG;
    }

    RAM[HL] = (RAM[HL] >> 1);

    if ((oldValue & 0b10000000) != 0) {
        RAM[HL] |= 0b10000000;
    }

    if (RAM[HL] == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SWAP_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F = NO_FLAG;

    R = (R << 4) | (R >> 4);

    if (R == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SWAP_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F = NO_FLAG;

    uint16_t HL = (H << 8) | L;

    RAM[HL] = (RAM[HL] << 4) | (RAM[HL] >> 4);

    if (RAM[HL] == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SRL_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F = NO_FLAG;

    if ((R & 0b00000001) != 0) {
        F |= CARRY_FLAG;
    }

    R = (R >> 1);

    if (R == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SRL_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F = NO_FLAG;

    uint16_t HL = (H << 8) | L;

    if ((RAM[HL] & 0b00000001) != 0) {
        F |= CARRY_FLAG;
    }

    RAM[HL] = (RAM[HL] >> 1);

    if (RAM[HL] == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void BIT_N_R(uint16_t& PC, uint8_t& R, uint8_t& F, int N) {
    F &= ~(ZERO_FLAG | NEGATIVE_FLAG);
    F |= HALF_CARRY_FLAG;

    uint8_t value = (R >> N);

    if ((value & 0b00000001) != 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void BIT_N_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F, int N) {
    F &= ~(ZERO_FLAG | NEGATIVE_FLAG);
    F |= HALF_CARRY_FLAG;

    uint16_t HL = (H << 8) | L;

    uint8_t value = (RAM[HL] >> N);

    if ((value & 0b00000001) != 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void RES_N_R(uint16_t& PC, uint8_t& R, int N) {
    R &= ~(1u << N);
}

void RES_N_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, int N) {
    uint16_t HL = (H << 8) | L;

    RAM[HL] &= ~(1u << N);
}