#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdint>

#include "../include/instruction.hpp"
#include "../include/const.hpp"
#include "../include/tools.hpp"

void NOP(uint16_t& PC) {
    std::cout << "Should sleep during one M-cycle" << std::endl;
    PC++;
}

void STOP_8(uint16_t& PC) {
    std::cout << "8 bits are important for GBC" << std::endl;
    PC += 2;
}

void HALT(uint16_t& PC) {
    std::cout << "CPU should stop" << std::endl;
}

void LD_R8_8(uint16_t& PC, uint8_t& R, std::vector<uint8_t>& RAM) {
    PC++;
    R = RAM[PC];

    PC++;
}

void LD_R8_ADR16(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t R21, uint8_t R22) {
    uint16_t AD16 = (R21 << 8) | R22;
    R1 = RAM[AD16];

    PC++;
}

void LD_R8_R8(uint16_t& PC, uint8_t& R1, uint8_t R2) {
    R1 = R2;

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
    uint16_t AD16 = (R11 << 8) | R12;
    RAM[AD16] = R2;

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

void LD_SP_16(uint16_t& PC, uint16_t& SP, std::vector<uint8_t>& RAM) {
    PC++;
    uint16_t value = RAM[PC];

    PC++;
    value |= (RAM[PC] << 8);

    SP = value;

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

void LD_ADHL_R8(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t R) {
    uint16_t HL = (H << 8) | L;

    RAM[HL] = R;

    PC++;
}

void LD_ADHL_8(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L) {
    PC++;
    uint16_t ADHL = (H << 8) | L;
    RAM[ADHL] = RAM[PC];

    PC++;
}

void LD_HL_SP8(uint16_t& PC, uint8_t& H, uint8_t& L, uint16_t SP, std::vector<uint8_t>& RAM, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    PC++;
    int8_t value = RAM[PC];

    // half-carry check
    if ((SP & 0xF) + ((uint8_t)value & 0xF) > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if ((SP & 0xFF) + (uint8_t)value > 0xFF) {
        F |= CARRY_FLAG;
    }

    H = (SP + value) >> 8;
    L = (SP + value) & 0xFF;

    PC++;
}

void LD_SP_HL(uint16_t& PC, uint16_t& SP, uint8_t H, uint8_t L) {
    SP = (H << 8) | L;
}

void LD_A_AD16(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM) {
    PC++;
    uint16_t address = RAM[PC];

    PC++;
    address |= RAM[PC] << 8;

    A = RAM[address];

    PC++;
}

void LDH_AD8_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t A) {
    PC++;
    uint8_t address = RAM[PC];

    RAM[0xFF00 | address] = A;

    PC++;
}

void LDH_ADC_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t C, uint8_t A) {
    RAM[0xFF00 | C] = A;

    PC++;
}

void LDH_A_AD8(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM) {
    PC++;
    uint8_t address = RAM[PC];

    A = RAM[0xFF00 | address];

    std::cout << "0x" << std::hex <<  +RAM[0xFF00 | address] << std::endl;; 

    PC++;
}

void LDH_A_ADC(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM, uint8_t C) {
    A = RAM[0xFF | C];

    PC++;
}

void INC_R8(uint16_t& PC, uint8_t& R, uint8_t& F) {
    // unset zero, negative and half-carry flags
    F &= ~(ZERO_FLAG | NEGATIVE_FLAG | HALF_CARRY_FLAG); 

    // half-carry check
    if ((R & 0x0F) == 0x0F) {
        F |= HALF_CARRY_FLAG;
    }

    R++;

    // zero check
    if (R == 0) {
        F |= ZERO_FLAG;
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
    // unset zero, negative and half-carry flags
    F &= ~(ZERO_FLAG | NEGATIVE_FLAG | HALF_CARRY_FLAG); 

    uint16_t combination = (H << 8) | L;

    // half-carry check
    if ((RAM[combination] & 0x0F) == 0x0F) {
        F |= HALF_CARRY_FLAG;
    }

    RAM[combination]++;

    // zero check
    if (RAM[combination] == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void INC_SP(uint16_t& PC, uint16_t& SP) {
    SP++;

    PC++;
}

void DEC_R8(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F |= NEGATIVE_FLAG; // set negative flag
    F &= (NEGATIVE_FLAG | CARRY_FLAG); // unset zero and half-carry flags

    // half-carry check
    if ((R & 0x0F) == 0) {
        F |= HALF_CARRY_FLAG;
    }

    R--;

    // zero check
    if (R == 0) {
        F |= ZERO_FLAG;
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
    F |= NEGATIVE_FLAG; // set negative flag
    F &= (NEGATIVE_FLAG | CARRY_FLAG); // unset zero and half-carry flags

    uint16_t combination = (H << 8) | L;

    // half-carry check
    if ((RAM[combination] & 0x0F) == 0) {
        F |= HALF_CARRY_FLAG;
    }

    RAM[combination]--;

    // zero check
    if (RAM[combination] == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void DEC_SP(uint16_t& PC, uint16_t& SP) {
    SP--;

    PC++;
}

void ADD_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    // half-carry check
    if ((R1 & 0xF) + (R2 & 0xF) > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (R1 + R2 > 0xFF) {
        F |= CARRY_FLAG;
    }

    R1 += R2;

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void ADD_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    uint16_t HL = (H << 8) | L;

    // half-carry check
    if ((R1 & 0xF) + (RAM[HL] & 0xF) > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (R1 + RAM[HL] > 0xFF) {
        F |= CARRY_FLAG;
    }

    R1 += RAM[HL];

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void ADD_R16_R16(uint16_t& PC, uint8_t& R11, uint8_t& R12, uint8_t R21, uint8_t R22, uint8_t& F) {
    // unset negative, half-carry and carry flags
    F &= ~(NEGATIVE_FLAG | HALF_CARRY_FLAG | CARRY_FLAG);

    uint16_t combinationR1 = (R11 << 8) | R12;
    uint16_t combinationR2 = (R21 << 8) | R22;

    // half-carry check
    if ((combinationR1 & 0xFFF) + (combinationR2 & 0xFFF) > 0xFFF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (combinationR1 + combinationR2 > 0xFFFF) {
        F |= CARRY_FLAG;
    }

    combinationR1 += combinationR2;
    R11 = combinationR1 >> 8;
    R22 = (combinationR2 & 0xFF);

    PC++;
}

void ADD_A_8(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    PC++;
    uint8_t value = RAM[PC];

    // half-carry check
    if ((A & 0xF) + (value & 0xF) > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (A + value > 0xF) {
        F |= CARRY_FLAG;
    }

    A += value;

    // zero check
    if (A == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void ADD_SP_8(uint16_t& PC, uint16_t& SP, std::vector<uint8_t>& RAM) {
    PC++;
    int8_t value = RAM[PC];

    SP += value;

    PC++;
}

void ADD_HL_SP(uint16_t& PC, uint8_t& H, uint8_t& L, uint16_t SP, uint8_t& F) {
    // unset negative, half-carry and carry flags
    F &= ~(NEGATIVE_FLAG | HALF_CARRY_FLAG | CARRY_FLAG);

    uint16_t HL = (H << 8) | L;

    // half-carry check
    if ((HL & 0xFFF) + (SP & 0xFFF) > 0xFFF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (HL + SP > 0xFFFF) {
        F |= CARRY_FLAG;
    }

    HL += SP;

    H = (HL >> 8);
    L = (HL & 0xFF);

    PC++;
}

void ADC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= NO_FLAG; // unset flags
    int carry = 0;

    if ((F & CARRY_FLAG) != 0) {
        carry = 1;
    }

    // half-carry check
    if ((R1 & 0xF) + (R2 & 0xF) + carry > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (R1 + R2 + carry > 0xFF) {
        F |= CARRY_FLAG;
    }

    R1 += R2 + carry;

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void ADC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= NO_FLAG; // unset flags
    int carry = 0;

    if ((F & CARRY_FLAG) != 0) {
        carry = 1;
    }

    uint16_t HL = (H << 8) | L;

    // half-carry check
    if ((R1 & 0xF) + (RAM[HL] & 0xF) + carry > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (R1 + RAM[HL] + carry > 0xFF) {
        F |= CARRY_FLAG;
    }

    R1 += RAM[HL];

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void ADC_A_8(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM, uint8_t& F) {
    F &= NO_FLAG; // unset flags
    int carry = 0;

    if ((F & CARRY_FLAG) != 0) {
        carry = 1;
    }

    PC++;

    // half-carry check
    if ((A & 0xF) + (RAM[PC] & 0xF) + carry > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (A + RAM[PC] + carry > 0xFF) {
        F |= CARRY_FLAG;
    }

    A += RAM[PC];

    // zero check
    if (A == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SUB_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F = NEGATIVE_FLAG; // set negative flag

    // half-carry check
    if ((R1 & 0xF) - (R2 & 0xF) < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (R1 - R2 < 0) {
        F |= CARRY_FLAG;
    }

    R1 -= R2;

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SUB_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F = NEGATIVE_FLAG; // set negative flag

    uint16_t HL = (H << 8) | L;

    // half-carry check
    if ((R1 & 0xF) - (RAM[HL] & 0xF) < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (R1 - RAM[HL] < 0) {
        F |= CARRY_FLAG;
    }

    R1 -= RAM[HL];

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SUB_A_8(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM, uint8_t& F) {
    F = NEGATIVE_FLAG; // set negative flag

    PC++;

    // half-carry check
    if ((A & 0xF) - (RAM[PC] & 0xF) < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (A - RAM[PC] < 0) {
        F |= CARRY_FLAG;
    }

    A -= RAM[PC];

    if (A == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SBC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F = NEGATIVE_FLAG; // set negative flag

    int carry = 0;

    if ((F & CARRY_FLAG) != 0) {
        carry = 1;
    }

    // half-carry check
    if ((R1 & 0xF) - (R2 & 0xF) - carry < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (R1 - R2 - carry < 0) {
        F |= CARRY_FLAG;
    }

    R1 -= (R2 + carry);

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SBC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F = NEGATIVE_FLAG; // set negative fla

    int carry = 0;

    if ((F & CARRY_FLAG) != 0) {
        carry = 1;
    }

    uint16_t HL = (H << 8) | L;

    // half-carry check
    if ((R1 & 0xF) - (RAM[HL] & 0xF) - carry < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (R1 - RAM[HL] - carry < 0) {
        F |= CARRY_FLAG;
    }

    R1 -= (RAM[HL] + carry);

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SBC_A_8(uint16_t& PC, uint8_t&A, std::vector<uint8_t>& RAM, uint8_t& F) {
    F = NEGATIVE_FLAG; // set negative flag

    int carry = 0;

    if ((F & CARRY_FLAG) != 0) {
        carry = 1;
    }

    PC++;

    // half-carry check
    if ((A & 0xF) - (RAM[PC] & 0xF) - carry < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (A - RAM[PC] - carry < 0) {
        F |= CARRY_FLAG;
    }

    A -= (RAM[PC] + carry);

    // zero check
    if (A == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void AND_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F = HALF_CARRY_FLAG; // set half-carry flag

    R1 &= R2;

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void AND_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F) {
    F = HALF_CARRY_FLAG; // set half-carry flag

    uint16_t HL = (H << 8) | L;

    R1 &= RAM[HL];

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void AND_A_8(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM, uint8_t& F) {
    F = HALF_CARRY_FLAG; // set half-carry flag

    PC++;
    A &= RAM[PC];

    PC++;
}

void XOR_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= NO_FLAG; // unset flags
    
    R1 ^= R2;

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void XOR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F) {
    F &= NO_FLAG; // unset flags

    uint16_t HL = (H << 8) | L;

    R1 ^= RAM[HL];

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void XOR_A_8(uint16_t& PC, uint8_t A, std::vector<uint8_t>& RAM, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    PC++;
    uint8_t value = RAM[PC];

    A ^= value;

    // zero check
    if (A == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void OR_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    R1 |= R2;

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void OR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F) {
    F &= NO_FLAG; // unset flags

    uint16_t HL = (H << 8) | L;

    R1 |= RAM[HL];

    // zero check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void OR_A_8(uint16_t& PC, uint8_t A, std::vector<uint8_t>& RAM, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    PC++;
    uint8_t value = RAM[PC];

    A |= value;

    // zero check
    if (A == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void CP_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F = NEGATIVE_FLAG; // set negative flag

    // half-carry check
    if ((R1 & 0xF) - (R2 & 0xF) < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (R1 - R2 < 0) {
        F |= CARRY_FLAG;
    }

    // zero check
    if (R1 == R2) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void CP_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F = NEGATIVE_FLAG; // set negative flag

    uint16_t HL = (H << 8) | L;

    // half-carry check
    if ((R1 & 0xF) - (RAM[HL] & 0xF) < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (R1 - RAM[HL] < 0) {
        F |= CARRY_FLAG;
    }

    // zero check
    if (R1 == RAM[HL]) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void CP_A_8(uint16_t& PC, uint8_t A, std::vector<uint8_t>& RAM, uint8_t& F) {
    F = NEGATIVE_FLAG; // set negative flag

    PC++;
    uint8_t value = RAM[PC];

    // half-carry check
    if ((A & 0xF) - (value & 0xF) < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry check
    if (A - value < 0) {
        F |= CARRY_FLAG;
    }

    // zero check
    if (A == value) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void RLCA(uint16_t& PC, uint8_t& A, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    uint8_t oldA = A;

    A = (A << 1);
    
    if ((oldA & 0b10000000) != 0) {
        F |= CARRY_FLAG; // set carry flag
        A |= 0b00000001;
    }

    PC ++;
}

void RRCA(uint16_t& PC, uint8_t& A, uint8_t& F) {
    F &= NO_FLAG; // unset flags

    uint8_t oldA = A;

    A = (A >> 1);

    if ((oldA & 0b00000001) != 0) {
        F |= CARRY_FLAG; // set carry flag
        A |= 0b10000000;
    }

    PC++;
}

void RLA(uint16_t& PC, uint8_t& A, uint8_t& F) {
    uint8_t oldA = A;
    
    A = (A << 1);
    
    // put carry in bit 0
    if ((F & CARRY_FLAG) != 0) {
        A |= 0b00000001;
    }
    
    F = NO_FLAG;

    // Put MSB of A into the carry
    if ((oldA & 0b10000000) != 0) {
        F |= CARRY_FLAG;
    }

    PC++;
}

void RRA(uint16_t& PC, uint8_t& A, uint8_t& F) {
    uint8_t oldA = A;

    A = (A >> 1);

    // put carry in bit 7
    if ((F & CARRY_FLAG) != 0) {
        A |= 0b10000000;
    }

    F &= NO_FLAG;

    // Put LSB of A into carry
    if ((oldA & 0b00000001) != 0) {
        F |= CARRY_FLAG;
    }


    PC++;
}

void CPL(uint16_t& PC, uint8_t& R, uint8_t& F) {
    // set negative and half-carry flag
    F |= (NEGATIVE_FLAG | HALF_CARRY_FLAG);

    R ^= 0xFF;

    PC++;
}

void SCF(uint16_t& PC, uint8_t& F) {
    F &= ZERO_FLAG; // unset flags except zero
    F |= CARRY_FLAG; // set carry flag

    PC++;
}

void CCF(uint16_t& PC, uint8_t& F) {
    F &= ZERO_FLAG; // unset flags except zero
    F ^= CARRY_FLAG; // set carry flag

    PC++;
}

void JR_8(uint16_t& PC, std::vector<uint8_t>& RAM) {
    PC++;
    uint8_t value = RAM[PC];

    PC++;

    PC += (signed char)value;
}

void JR_Z_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM) {
    PC++;
    uint8_t value = RAM[PC];

    PC++;

    if ((F & ZERO_FLAG) != 0) {
        PC += (signed char)value;
    }
}

void JR_NZ_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM) {
    PC++;
    uint8_t value = RAM[PC];

    PC++;

    if ((F & ZERO_FLAG) == 0) {
        PC += (signed char)value;
    }
}

void JR_C_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM) {
    PC++;
    uint8_t value = RAM[PC];

    PC++;

    if ((F & CARRY_FLAG) != 0) {
        PC += (signed char)value;
    }
}

void JR_NC_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM) {
    PC++;
    uint8_t value = RAM[PC];

    PC++;

    if ((F & CARRY_FLAG) == 0) {
        PC += (signed char)value;
    }
}

void DAA(uint16_t& PC, uint8_t& A, uint8_t& F) {
    // unset flags except negative
    F &= ~(ZERO_FLAG | HALF_CARRY_FLAG | CARRY_FLAG);

    int8_t MSBadjust = 0x60;
    int8_t LSBadjust = 0x06;

    uint8_t oldA = A;

    bool setCarry = false;

    if ((F & NEGATIVE_FLAG) != 0) {
        MSBadjust = -MSBadjust;
        LSBadjust = -LSBadjust;
    }
    
    if ((F & CARRY_FLAG) != 0) {
        A += MSBadjust;
        setCarry = true;
    }
    
    if ((F & HALF_CARRY_FLAG) != 0) {
        A += LSBadjust;
    }
    
    for (int i = 0; i < NB_DAA_CHECK; i++) {
        if ((A & 0xF) > 0x9) {
            A += LSBadjust;
        }
        
        if ((A & 0xF0) > 0x90) {
            A += MSBadjust;
        }
    }
    
    if (A < oldA || setCarry) {
        F |= CARRY_FLAG;
    } else {
        F &= ~CARRY_FLAG;
    }
    
    if (A == 0) {
        F |= ZERO_FLAG;
    }
    
    PC++;
}

void RET(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP) {
    uint16_t address = RAM[SP];
    SP++;

    address |= (RAM[SP] << 8);
    SP++;

    PC = address;
}

void RET_Z(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F) {
    if ((F & ZERO_FLAG) == 1) {
        uint16_t address = RAM[SP];
        SP++;

        address |= (RAM[SP] << 8);
        SP++;

        PC = address;
    }
}

void RET_NZ(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t &SP, uint8_t F) {
    if ((F & ZERO_FLAG) == 0) {
        uint16_t address = RAM[SP];
        SP++;

        address |= (RAM[SP] << 8);
        SP++;

        PC = address;
    }
}

void RET_C(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F) {
    if ((F & CARRY_FLAG) == 1) {
        uint16_t address = RAM[SP];
        SP++;

        address |= (RAM[SP] << 8);
        SP++;

        PC = address;
    }
}

void RET_NC(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F) {
    if ((F & CARRY_FLAG) == 0) {
        uint16_t address = RAM[SP];
        SP++;

        address |= (RAM[SP] << 8);
        SP++;

        PC = address;
    }
}

void RETI(uint16_t& PC) {
    std::cout << "Should return from interruption" << std::endl;
}

void PUSH_R16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t& R1, uint8_t& R2) {
    RAM[SP] = R2;
    SP--;

    RAM[SP] = R1;
    SP--;

    PC++;
}

void POP_R16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t& R1, uint8_t& R2) {
    R2 = RAM[SP];
    SP++;

    R1 = RAM[SP];
    SP++;

    PC++;
}

void JP_16(uint16_t& PC, std::vector<uint8_t>& RAM) {
    PC++;
    uint16_t address = RAM[PC];

    PC++;
    address |= (RAM[PC] << 8);

    PC = address;
}

void JP_Z_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t F) {
    if ((F & ZERO_FLAG) == 1) {
        PC++;
        uint16_t address = RAM[PC];

        PC++;
        address |= (RAM[PC] << 8);

        PC = address;
    }
}

void JP_NZ_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t F) {
    if ((F & ZERO_FLAG) == 0) {
        PC++;
        uint16_t address = RAM[PC];

        PC++;
        address |= (RAM[PC] << 8);

        PC = address;
    }
}

void JP_C_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t F) {
    if ((F & CARRY_FLAG) == 1) {
        PC++;
        uint16_t address = RAM[PC];

        PC++;
        address |= (RAM[PC] << 8);

        PC = address;
    }
}

void JP_NC_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t F) {
    if ((F & CARRY_FLAG) == 0) {
        PC++;
        uint16_t address = RAM[PC];

        PC++;
        address |= (RAM[PC] << 8);

        PC = address;
    }
}

void JP_HL(uint16_t& PC, uint8_t H, uint8_t L) {
    uint16_t HL = (H << 8) | L;

    PC = HL;
}

void RST_AD(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t AD) {
    RAM[SP] = (PC + 1) >> 8;
    SP--;
    RAM[SP] = (PC + 1) & 0xFF;
    SP--;

    PC = AD;
}

void CALL_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP) {
    RAM[SP] = (PC + 3) >> 8;
    SP--;
    RAM[SP] = (PC + 3) & 0xFF;
    SP--;

    PC++;
    uint16_t address = RAM[PC];

    PC++;
    address |= (RAM[PC] << 8);

    PC = address;
}

void CALL_Z_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F) {
    if ((F & ZERO_FLAG) == 1) {
        RAM[SP] = (PC + 3) >> 8;
        SP--;
        RAM[SP] = (PC + 3) & 0xFF;
        SP--;

        PC++;
        uint16_t address = RAM[PC];

        PC++;
        address |= (RAM[PC] << 8);

        PC = address;
    }
}

void CALL_NZ_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F) {
    if ((F & ZERO_FLAG) == 0) {
        RAM[SP] = (PC + 3) >> 8;
        SP--;
        RAM[SP] = (PC + 3) & 0xFF;
        SP--;

        PC++;
        uint16_t address = RAM[PC];

        PC++;
        address |= (RAM[PC] << 8);

        PC = address;
    }
}

void CALL_C_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F) {
    if ((F & CARRY_FLAG) == 1) {
        RAM[SP] = (PC + 3) >> 8;
        SP--;
        RAM[SP] = (PC + 3) & 0xFF;
        SP--;

        PC++;
        uint16_t address = RAM[PC];

        PC++;
        address |= (RAM[PC] << 8);

        PC = address;
    }
}

void CALL_NC_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F) {
    if ((F & CARRY_FLAG) == 0) {
        RAM[SP] = (PC + 3) >> 8;
        SP--;
        RAM[SP] = (PC + 3) & 0xFF;
        SP--;

        PC++;
        uint16_t address = RAM[PC];

        PC++;
        address |= (RAM[PC] << 8);

        PC = address;
    }
}

void PREFIX(uint16_t& PC) {
    // will be implemented with prefixed instructions
}