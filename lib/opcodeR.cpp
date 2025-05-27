#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdint>

#include "../include/opcodeR.hpp"

void NOP(uint16_t& PC) {
    std::cout << "Should sleep during one M-cycle" << std::endl;
    PC++;
}

void STOP_8(uint16_t& PC) {
    std::cout << "8 bits are important for GBC" << std::endl;
    PC++;
    PC++;
}

void HALT(uint16_t& PC) {
    std::cout << "CPU should stop" << std::endl;
}

void LD_R_8(uint16_t& PC, uint8_t& R, std::vector<uint8_t>& RAM) {
    PC++;
    R = RAM[PC];

    PC++;
}

void LD_R_ADR16(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t R21, uint8_t R22) {
    uint16_t ADR16 = (R21 << 8) | R22;
    R1 = RAM[ADR16];

    PC++;
}

void LD_R16_16(uint16_t& PC, uint8_t& R1, uint8_t& R2, std::vector<uint8_t>& RAM) {
    PC++;
    R2 = RAM[PC];
    PC++;
    R1= RAM[PC];

    PC++;
}

void LD_ADR16_R(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t R11, uint8_t R12, uint8_t R2) {
    uint16_t ADR16 = (R11 << 8) | R12;
    RAM[ADR16] = R2;

    PC++;
}

void LD_AD16_SP(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t SP) {
    PC++;
    uint16_t AD16 = (RAM[PC] << 8);
    PC++;
    AD16 |= RAM[PC];

    RAM[AD16] = (SP & 0xFF);
    AD16++;
    RAM[AD16] = (SP >> 8);

    PC++;
}

void LD_AD16_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t A) {
    PC++;
    uint16_t AD16 = (RAM[PC] << 8);
    PC++;
    AD16 |= RAM[PC];

    RAM[AD16] = A;

    PC++;
}

void LD_ADHL_I_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A) {
    uint16_t ADHL = (H << 8) | L;
    RAM[ADHL] = A;

    ADHL++;
    H = (ADHL >> 8);
    L = (ADHL & 0xFF);

    PC++;
}

void LD_ADHL_D_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A) {
    uint16_t ADHL = (H << 8) | L;
    RAM[ADHL] = A;

    ADHL--;
    H = (ADHL >> 8);
    L = (ADHL & 0xFF);

    PC++;
}

void LD_A_ADHL_I(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A) {
    uint16_t ADHL = (H << 8) | L;
    A = RAM[ADHL];

    ADHL++;
    H = (ADHL >> 8);
    L = (ADHL & 0xFF);

    PC++;
}

void LD_A_ADHL_D(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A) {
    uint16_t ADHL = (H << 8) | L;
    A = RAM[ADHL];

    ADHL--;
    H = (ADHL >> 8);
    L = (ADHL & 0xFF);

    PC++;
}

void LD_ADHL_8(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L) {
    PC++;
    uint16_t ADHL = (H << 8) | L;
    RAM[ADHL] = RAM[PC];

    PC++;
}

void INC_R8(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F &= 0b10111111; // unset negative flag

    // half-carry flag check
    if ((R & 0x0F) == 0x0F) {
        F |= 0b00100000;
    } else {
        F &= 0b11011111;
    }

    R++;

    // zero flag check
    if (R == 0) {
        F |= 0b10000000;
    } else {
        F &= 0b01111111;
    }

    PC++;
}

void INC_R16(uint16_t& PC, uint8_t& R1, uint8_t& R2) {
    uint16_t combination = (R1 << 8) | R2;
    
    combination++;
    R1 = (combination >> 8);
    R2 = (combination & 0xFF);

    PC++;
}

void INC_ADHL(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= 0b10111111; // unset negative flag

    uint16_t combination = (H << 8) | L;

    // half-carry flag check
    if ((RAM[combination] & 0x0F) == 0x0F) {
        F |= 0b00100000;
    } else {
        F &= 0b11011111;
    }

    RAM[combination]++;

    // zero flag check
    if (RAM[combination] == 0) {
        F |= 0b10000000;
    } else {
        F &= 0b01111111;
    }

    PC++;
}

void DEC_R8(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F |= 0b01000000; // set negative flag

    // half-carry flag check
    if ((R & 0x0F) == 0) {
        F |= 0b00100000;
    } else {
        F &= 0b11011111;
    }

    R--;

    // zero flag check
    if (R == 0) {
        F |= 0b10000000;
    } else {
        F &= 0b01111111;
    }
    
    PC++;
}

void DEC_R16(uint16_t& PC, uint8_t& R1, uint8_t& R2) {
    uint16_t combination = (R1 << 8) | R2;
    
    combination--;
    R1 = (combination >> 8);
    R2 = (combination & 0xFF);

    PC++;
}

void DEC_ADHL(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= 0b10111111; // unset negative flag

    uint16_t combination = (H << 8) | L;

    // half-carry flag check
    if ((RAM[combination] & 0x0F) == 0) {
        F |= 0b00100000;
    } else {
        F &= 0b11011111;
    }

    RAM[combination]--;

    // zero flag check
    if (RAM[combination] == 0) {
        F |= 0b10000000;
    } else {
        F &= 0b01111111;
    }

    PC++;
}

void ADD_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= 0b10111111; // unset negative flag

    // half-carry flag check
    if ((R1 & 0xF) + (R2 & 0xF) > 0xF) {
        F |= 0b00100000;
    } else {
        F &= 0b11011111;
    }

    // carry flag check
    if (R1 + R2 > 0xFF) {
        F |= 0b00010000;
    } else {
        F &= 0b11101111;
    }

    R1 += R2;

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    } else {
        F &= 0b01111111;
    }

    PC++;
}

void ADD_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= 0b10111111; // unset negative flag

    PC++;

    // half-carry flag check
    if ((R1 & 0xF) + (RAM[PC] & 0xF) > 0xF) {
        F |= 0b00100000;
    } else {
        F &= 0b11011111;
    }

    // carry flag check
    if (R1 + RAM[PC] > 0xFF) {
        F |= 0b00010000;
    } else {
        F &= 0b11101111;
    }

    R1 += RAM[PC];

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    } else {
        F &= 0b01111111;
    }

    PC++;
}

void ADD_R16_R16(uint16_t& PC, uint8_t& R11, uint8_t& R12, uint8_t R21, uint8_t R22, uint8_t& F) {
    F &= 0b10111111; // unset negative flag

    uint16_t combinationR1 = (R11 << 8) | R12;
    uint16_t combinationR2 = (R21 << 8) | R22;

    // half-carry check
    if ((combinationR1 & 0xFFF) + (combinationR2 & 0xFFF) > 0xFFF) {
        F |= 0b00100000;
    } else {
        F &= 0b11011111;
    }

    // carry flag check
    if (combinationR1 + combinationR2 > 0xFFFF) {
        F |= 0b00010000;
    } else {
        F &= 0b11101111;
    }

    combinationR1 += combinationR2;
    R11 = combinationR1 >> 8;
    R22 = (combinationR2 & 0xFF);

    PC++;
}


void ADC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= 0b10111111; // unset negative flag
    int carry = 0;

    if ((F & 0b00010000) != 0) {
        carry = 1;
    }

    // half-carry flag check
    if ((R1 & 0xF) + (R2 & 0xF) + carry > 0xF) {
        F |= 0b00100000;
    } else {
        F &= 0b11011111;
    }

    // carry flag check
    if (R1 + R2 + carry > 0xFF) {
        F |= 0b00010000;
    } else {
        F &= 0b11101111;
    }

    R1 += R2 + carry;

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    } else {
        F &= 0b01111111;
    }

    PC++;
}

void ADC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= 0b10111111; // unset negative flag
    int carry = 0;

    if ((F & 0b00010000) != 0) {
        carry = 1;
    }

    PC++;

    // half-carry flag check
    if ((R1 & 0xF) + (RAM[PC] & 0xF) + carry > 0xF) {
        F |= 0b00100000;
    } else {
        F &= 0b11011111;
    }

    // carry flag check
    if (R1 + RAM[PC] + carry > 0xFF) {
        F |= 0b00010000;
    } else {
        F &= 0b11101111;
    }

    R1 += RAM[PC];

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    } else {
        F &= 0b01111111;
    }

    PC++;
}

void SUB_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F |= 0b01000000;

    
}