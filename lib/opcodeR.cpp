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
    F &= 0b00011111; // unset zero, negative and half-carry flags

    // half-carry flag check
    if ((R & 0x0F) == 0x0F) {
        F |= 0b00100000;
    }

    R++;

    // zero flag check
    if (R == 0) {
        F |= 0b10000000;
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
    F &= 0b00011111; // unset zero, negative and half-carry flags

    uint16_t combination = (H << 8) | L;

    // half-carry flag check
    if ((RAM[combination] & 0x0F) == 0x0F) {
        F |= 0b00100000;
    }

    RAM[combination]++;

    // zero flag check
    if (RAM[combination] == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void DEC_R8(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F |= 0b01000000; // set negative flag
    F &= 0b01000000; // unset zero and half-carry flags

    // half-carry flag check
    if ((R & 0x0F) == 0) {
        F |= 0b00100000;
    }

    R--;

    // zero flag check
    if (R == 0) {
        F |= 0b10000000;
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
    F |= 0b01000000; // set negative flag
    F &= 0b01000000; // unset zero and half-carry flags

    uint16_t combination = (H << 8) | L;

    // half-carry flag check
    if ((RAM[combination] & 0x0F) == 0) {
        F |= 0b00100000;
    }

    RAM[combination]--;

    // zero flag check
    if (RAM[combination] == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void ADD_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= 0b00001111; // unset flags

    // half-carry flag check
    if ((R1 & 0xF) + (R2 & 0xF) > 0xF) {
        F |= 0b00100000;
    }

    // carry flag check
    if (R1 + R2 > 0xFF) {
        F |= 0b00010000;
    }

    R1 += R2;

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void ADD_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= 0b00001111; // unset flags

    uint16_t HL = (H << 8) | L;

    // half-carry flag check
    if ((R1 & 0xF) + (RAM[HL] & 0xF) > 0xF) {
        F |= 0b00100000;
    }

    // carry flag check
    if (R1 + RAM[HL] > 0xFF) {
        F |= 0b00010000;
    }

    R1 += RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void ADD_R16_R16(uint16_t& PC, uint8_t& R11, uint8_t& R12, uint8_t R21, uint8_t R22, uint8_t& F) {
    F &= 0b10001111; // unset negative, half-carry and carry flags

    uint16_t combinationR1 = (R11 << 8) | R12;
    uint16_t combinationR2 = (R21 << 8) | R22;

    // half-carry check
    if ((combinationR1 & 0xFFF) + (combinationR2 & 0xFFF) > 0xFFF) {
        F |= 0b00100000;
    }

    // carry flag check
    if (combinationR1 + combinationR2 > 0xFFFF) {
        F |= 0b00010000;
    }

    combinationR1 += combinationR2;
    R11 = combinationR1 >> 8;
    R22 = (combinationR2 & 0xFF);

    PC++;
}


void ADC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= 0b00001111; // unset flags
    int carry = 0;

    if ((F & 0b00010000) != 0) {
        carry = 1;
    }

    // half-carry flag check
    if ((R1 & 0xF) + (R2 & 0xF) + carry > 0xF) {
        F |= 0b00100000;
    }

    // carry flag check
    if (R1 + R2 + carry > 0xFF) {
        F |= 0b00010000;
    }

    R1 += R2 + carry;

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void ADC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= 0b00001111; // unset flags
    int carry = 0;

    if ((F & 0b00010000) != 0) {
        carry = 1;
    }

    uint16_t HL = (H << 8) | L;

    // half-carry flag check
    if ((R1 & 0xF) + (RAM[HL] & 0xF) + carry > 0xF) {
        F |= 0b00100000;
    }

    // carry flag check
    if (R1 + RAM[HL] + carry > 0xFF) {
        F |= 0b00010000;
    }

    R1 += RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void SUB_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F |= 0b01000000; // set negative flag
    F &= 0b01000000; // unset other flags

    // half-carry flag check
    if ((R1 & 0xF) - (R2 & 0xF) < 0) {
        F |= 0b00100000;
    }

    // carry flag check
    if (R1 - R2 < 0) {
        F |= 0b00010000;
    }

    R1 -= R2;

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void SUB_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F |= 0b01000000; // set negative flag
    F &= 0b01000000; // unset other flags

    uint16_t HL = (H << 8) | L;

    // half-carry flag check
    if ((R1 & 0xF) - (RAM[HL] & 0xF) < 0) {
        F |= 0b00100000;
    }

    // carry flag check
    if (R1 - RAM[HL] < 0) {
        F |= 0b00010000;
    }

    R1 -= RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void SBC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F |= 0b01000000; // set negative flag
    F &= 0b01000000; // unset other flags

    int carry = 0;

    if ((F & 0b00010000) != 0) {
        carry = 1;
    }

    // half-carry flag check
    if ((R1 & 0xF) - (R2 & 0xF) - carry < 0) {
        F |= 0b00100000;
    }

    // carry flag check
    if (R1 - R2 - carry < 0) {
        F |= 0b00010000;
    }

    R1 -= (R2 + carry);

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void SBC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F |= 0b01000000; // set negative flag
    F &= 0b01000000; // unset other flags

    int carry = 0;

    if ((F & 0b00010000) != 0) {
        carry = 1;
    }

    uint16_t HL = (H << 8) | L;

    // half-carry flag check
    if ((R1 & 0xF) - (RAM[HL] & 0xF) - carry < 0) {
        F |= 0b00100000;
    }

    // carry flag check
    if (R1 - RAM[HL] - carry < 0) {
        F |= 0b00010000;
    }

    R1 -= (RAM[HL] + carry);

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void AND_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F |= 0b00100000; // set half-carry flag
    F &= 0b00100000; // unset zero, negative and carry flags

    R1 &= R2;

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void AND_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F) {
    F |= 0b00100000; // set half-carry flag
    F &= 0b00100000; // unset zero, negative and carry flags

    uint16_t HL = (H << 8) | L;

    R1 &= RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void XOR_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= 0x00; // unset flags
    
    R1 ^= R2;

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void XOR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F) {
    F &= 0x00; // unset flags

    uint16_t HL = (H << 8) | L;

    R1 ^= RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void OR_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= 0x00; // unset flags

    R1 |= R2;

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void OR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F) {
    F &= 0x00; // unset flags

    uint16_t HL = (H << 8) | L;

    R1 |= RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void CP_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F |= 0b01000000; // set negative flag
    F &= 0b01000000; // unset other flags

    // half-carry flag check
    if ((R1 & 0xF) - (R2 & 0xF) < 0) {
        F |= 0b00100000;
    }

    // carry flag check
    if (R1 - R2 < 0) {
        F |= 0b00010000;
    }

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

void CP_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F |= 0b01000000; // set negative flag
    F &= 0b01000000; // unset other flags

    uint16_t HL = (H << 8) | L;

    // half-carry flag check
    if ((R1 & 0xF) - (RAM[HL] & 0xF) < 0) {
        F |= 0b00100000;
    }

    // carry flag check
    if (R1 - RAM[HL] < 0) {
        F |= 0b00010000;
    }

    // zero flag check
    if (R1 == 0) {
        F |= 0b10000000;
    }

    PC++;
}

// The entire function is not optimized
// A much better way should be found
void RLC_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F &= 0b00011111; // Unset zero negative and half-carry flags

    if ((R & 0b10000000) != 0) {
        F |= 0b00010000; // Set carry flag
        R = (R << 1) | 0xFF;
        R |= 0b00000001;
    } else {
        F &= 0b11101111;
        R = (R << 1) | 0xFF;
    }

    PC ++;
}

void RRC_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F &= 0b00011111; // Unset zero negative and half-carry flags

    if ((R & 0b00000001) != 0) {
        F |= 0b00010000; // Set carry flag
        R = R >> 1;
        R |= 0b10000000;
    } else {
        F &= 0b11101111;
        R = R >> 1;
    }

    PC++;
}

void RL_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F &= 0b00011111; // Unset zero negative and half-carry flags

    bool mst = false;

    if ((R & 0b10000000) != 0) {
        mst = true;
    }
    
    R = (R << 1) & 0xFF;
    
    // If carry flag is set
    if ((F & 0b00010000) != 0) {
        R |= 0b00000001;
    }

    // Put MST of A into carry
    if (mst) {
        F |= 0b00010000;
    } else {
        F &= 0b11101111;
    }

    PC++;
}

void RR_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F &= 0b00011111; // Unset Zero negative and half-carry flags

    bool lst = false;

    if ((R & 0b00000001) != 0) {
        lst = true;
    }

    R = (R >> 1) & 0xFF;

    // If carry flag is set
    if ((F & 0b00010000) != 0) {
        R |= 0b10000000;
    }

    // Put MST of A into carry
    if (lst) {
        F |= 0b00010000;
    } else {
        F &= 0b11101111;
    }

    PC++;
}

void CPL(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F |= 0b01100000;

    R ^= 0xFF;

    PC++;
}

void SCF(uint16_t& PC, uint8_t& F) {
    F &= 0b10000000; // unset flags except zero
    F |= 0b00010000; // set carry flag

    PC++;
}

void CCF(uint16_t& PC, uint8_t& F) {
    F &= 0b10000000; // unset flags except zero
    F ^= 0b00010000; // set carry flag

    PC++;
}

void JR_8(uint16_t& PC, std::vector<uint8_t>& RAM) {
    PC++;
    u_int8_t value = RAM[PC];

    PC++;

    PC += (signed char)value;
}

void JR_NZ_8(u_int16_t& PC, u_int8_t& F, std::vector<u_int8_t>& RAM) {
    PC++;
    u_int8_t value = RAM[PC];

    PC++;

    if ((F & 0b10000000) == 0) {
        PC += (signed char)value;
    }
}

void JR_Z_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM) {
    PC++;
    u_int8_t value = RAM[PC];

    PC++;

    if ((F & 0b10000000) != 0) {
        PC += (signed char)value;
    }
}

void JR_NC_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM) {
    PC++;
    u_int8_t value = RAM[PC];

    PC++;

    if ((F & 0b00010000) == 0) {
        PC += (signed char)value;
    }
}

void JR_C_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM) {
    PC++;
    u_int8_t value = RAM[PC];

    PC++;

    if ((F & 0b00010000) != 0) {
        PC += (signed char)value;
    }
}