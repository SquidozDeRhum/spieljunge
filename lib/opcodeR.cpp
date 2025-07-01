#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdint>

#include "../include/opcodeR.hpp"
#include "../include/const.hpp"
#include "../include/tools.hpp"

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
    // unset zero, negative and half-carry flags
    F &= ~(ZERO_FLAG | NEGATIVE_FLAG | HALF_CARRY_FLAG); 

    // half-carry flag check
    if ((R & 0x0F) == 0x0F) {
        F |= HALF_CARRY_FLAG;
    }

    R++;

    // zero flag check
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

    // half-carry flag check
    if ((RAM[combination] & 0x0F) == 0x0F) {
        F |= HALF_CARRY_FLAG;
    }

    RAM[combination]++;

    // zero flag check
    if (RAM[combination] == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void DEC_R8(uint16_t& PC, uint8_t& R, uint8_t& F) {
    F |= NEGATIVE_FLAG; // set negative flag
    F &= (NEGATIVE_FLAG | CARRY_FLAG); // unset zero and half-carry flags

    // half-carry flag check
    if ((R & 0x0F) == 0) {
        F |= HALF_CARRY_FLAG;
    }

    R--;

    // zero flag check
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

    // half-carry flag check
    if ((RAM[combination] & 0x0F) == 0) {
        F |= HALF_CARRY_FLAG;
    }

    RAM[combination]--;

    // zero flag check
    if (RAM[combination] == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void ADD_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= 0x00; // unset flags

    // half-carry flag check
    if ((R1 & 0xF) + (R2 & 0xF) > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry flag check
    if (R1 + R2 > 0xFF) {
        F |= CARRY_FLAG;
    }

    R1 += R2;

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void ADD_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= 0x00; // unset flags

    uint16_t HL = (H << 8) | L;

    // half-carry flag check
    if ((R1 & 0xF) + (RAM[HL] & 0xF) > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry flag check
    if (R1 + RAM[HL] > 0xFF) {
        F |= CARRY_FLAG;
    }

    R1 += RAM[HL];

    // zero flag check
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

    // carry flag check
    if (combinationR1 + combinationR2 > 0xFFFF) {
        F |= CARRY_FLAG;
    }

    combinationR1 += combinationR2;
    R11 = combinationR1 >> 8;
    R22 = (combinationR2 & 0xFF);

    PC++;
}


void ADC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= 0x00; // unset flags
    int carry = 0;

    if ((F & CARRY_FLAG) != 0) {
        carry = 1;
    }

    // half-carry flag check
    if ((R1 & 0xF) + (R2 & 0xF) + carry > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry flag check
    if (R1 + R2 + carry > 0xFF) {
        F |= CARRY_FLAG;
    }

    R1 += R2 + carry;

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void ADC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F &= 0x00; // unset flags
    int carry = 0;

    if ((F & CARRY_FLAG) != 0) {
        carry = 1;
    }

    uint16_t HL = (H << 8) | L;

    // half-carry flag check
    if ((R1 & 0xF) + (RAM[HL] & 0xF) + carry > 0xF) {
        F |= HALF_CARRY_FLAG;
    }

    // carry flag check
    if (R1 + RAM[HL] + carry > 0xFF) {
        F |= CARRY_FLAG;
    }

    R1 += RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SUB_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F |= NEGATIVE_FLAG; // set negative flag
    F &= NEGATIVE_FLAG; // unset other flags

    // half-carry flag check
    if ((R1 & 0xF) - (R2 & 0xF) < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry flag check
    if (R1 - R2 < 0) {
        F |= CARRY_FLAG;
    }

    R1 -= R2;

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SUB_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F |= NEGATIVE_FLAG; // set negative flag
    F &= NEGATIVE_FLAG; // unset other flags

    uint16_t HL = (H << 8) | L;

    // half-carry flag check
    if ((R1 & 0xF) - (RAM[HL] & 0xF) < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry flag check
    if (R1 - RAM[HL] < 0) {
        F |= CARRY_FLAG;
    }

    R1 -= RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SBC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F |= NEGATIVE_FLAG; // set negative flag
    F &= NEGATIVE_FLAG; // unset other flags

    int carry = 0;

    if ((F & CARRY_FLAG) != 0) {
        carry = 1;
    }

    // half-carry flag check
    if ((R1 & 0xF) - (R2 & 0xF) - carry < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry flag check
    if (R1 - R2 - carry < 0) {
        F |= CARRY_FLAG;
    }

    R1 -= (R2 + carry);

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void SBC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F |= NEGATIVE_FLAG; // set negative flag
    F &= NEGATIVE_FLAG; // unset other flags

    int carry = 0;

    if ((F & CARRY_FLAG) != 0) {
        carry = 1;
    }

    uint16_t HL = (H << 8) | L;

    // half-carry flag check
    if ((R1 & 0xF) - (RAM[HL] & 0xF) - carry < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry flag check
    if (R1 - RAM[HL] - carry < 0) {
        F |= CARRY_FLAG;
    }

    R1 -= (RAM[HL] + carry);

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void AND_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F |= HALF_CARRY_FLAG; // set half-carry flag
    F &= HALF_CARRY_FLAG; // unset zero, negative and carry flags

    R1 &= R2;

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void AND_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F) {
    F |= HALF_CARRY_FLAG; // set half-carry flag
    F &= HALF_CARRY_FLAG; // unset zero, negative and carry flags

    uint16_t HL = (H << 8) | L;

    R1 &= RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void XOR_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= 0x00; // unset flags
    
    R1 ^= R2;

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void XOR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F) {
    F &= 0x00; // unset flags

    uint16_t HL = (H << 8) | L;

    R1 ^= RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void OR_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F &= 0x00; // unset flags

    R1 |= R2;

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void OR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F) {
    F &= 0x00; // unset flags

    uint16_t HL = (H << 8) | L;

    R1 |= RAM[HL];

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void CP_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F) {
    F |= NEGATIVE_FLAG; // set negative flag
    F &= NEGATIVE_FLAG; // unset other flags

    // half-carry flag check
    if ((R1 & 0xF) - (R2 & 0xF) < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry flag check
    if (R1 - R2 < 0) {
        F |= CARRY_FLAG;
    }

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

void CP_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F) {
    F |= NEGATIVE_FLAG; // set negative flag
    F &= NEGATIVE_FLAG; // unset other flags

    uint16_t HL = (H << 8) | L;

    // half-carry flag check
    if ((R1 & 0xF) - (RAM[HL] & 0xF) < 0) {
        F |= HALF_CARRY_FLAG;
    }

    // carry flag check
    if (R1 - RAM[HL] < 0) {
        F |= CARRY_FLAG;
    }

    // zero flag check
    if (R1 == 0) {
        F |= ZERO_FLAG;
    }

    PC++;
}

// The entire function is not optimized
// A much better way should be found
void RLC_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    // unset zero negative and half-carry flags
    F &= ~(ZERO_FLAG | NEGATIVE_FLAG | HALF_CARRY_FLAG); 

    if ((R & 0b10000000) != 0) {
        F |= CARRY_FLAG; // set carry flag
        R = (R << 1) | 0xFF;
        R |= 0b00000001;
    } else {
        F &= ~CARRY_FLAG; // unset carry flag
        R = (R << 1) | 0xFF;
    }

    PC ++;
}

void RRC_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    // unset zero negative and half-carry flags
    F &= ~(ZERO_FLAG | NEGATIVE_FLAG | HALF_CARRY_FLAG); 

    if ((R & 0b00000001) != 0) {
        F |= CARRY_FLAG; // set carry flag
        R = R >> 1;
        R |= 0b10000000;
    } else {
        F &= ~CARRY_FLAG; // unset carry flag
        R = R >> 1;
    }

    PC++;
}

void RL_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    // unset zero negative and half-carry flags
    F &= ~(ZERO_FLAG | NEGATIVE_FLAG | HALF_CARRY_FLAG); 

    bool msb = false;

    if ((R & 0b10000000) != 0) {
        msb = true;
    }
    
    R = (R << 1) & 0xFF;
    
    // If carry flag is set
    if ((F & CARRY_FLAG) != 0) {
        R |= 0b00000001;
    }

    // Put MSB of A into carry
    if (msb) {
        F |= CARRY_FLAG;
    } else {
        F &= ~CARRY_FLAG;
    }

    PC++;
}

void RR_R(uint16_t& PC, uint8_t& R, uint8_t& F) {
    // unset zero negative and half-carry flags
    F &= ~(ZERO_FLAG | NEGATIVE_FLAG | HALF_CARRY_FLAG); 

    bool lst = false;

    if ((R & 0b00000001) != 0) {
        lst = true;
    }

    R = (R >> 1) & 0xFF;

    // If carry flag is set
    if ((F & CARRY_FLAG) != 0) {
        R |= 0b10000000;
    }

    // Put LSB of A into carry
    if (lst) {
        F |= CARRY_FLAG;
    } else {
        F &= ~CARRY_FLAG;
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

void JR_NZ_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM) {
    PC++;
    uint8_t value = RAM[PC];

    PC++;

    if ((F & ZERO_FLAG) == 0) {
        PC += (signed char)value;
    }
}

void JR_Z_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM) {
    PC++;
    uint8_t value = RAM[PC];

    PC++;

    if ((F & ZERO_FLAG) != 0) {
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

void JR_C_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM) {
    PC++;
    uint8_t value = RAM[PC];

    PC++;

    if ((F & CARRY_FLAG) != 0) {
        PC += (signed char)value;
    }
}

void DAA(uint16_t& PC, uint8_t& A, uint8_t& F) {
    F &= ~ZERO_FLAG;

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
    
    F &= ~HALF_CARRY_FLAG;
    PC++;
}

void RET_NZ(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t &SP, uint8_t F) {
    if ((F & ZERO_FLAG) == 0) {
        uint16_t adress = RAM[SP];
        SP--;

        adress |= (RAM[SP] << 8);
        SP--;

        PC = adress;
    }
}

void POP_R16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t& R1, uint8_t& R2) {
    R2 = RAM[SP];
    SP--;

    R1 = RAM[SP];
    SP--;

    PC++;
}

void JP_NZ_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t F) {
    if ((F & ZERO_FLAG) == 0) {
        PC++;
        uint16_t adress = RAM[PC];

        PC++;
        adress |= (RAM[PC] << 8);

        PC = adress;
    }
}

void JP_16(uint16_t& PC, std::vector<uint8_t>& RAM) {
    PC++;
    uint16_t adress = RAM[PC];

    PC++;
    adress |= (RAM[PC] << 8);

    PC = adress;
}

void CALL_NZ_16(uint16_t& PC, uint16_t& SP,std::vector<uint8_t>& RAM, uint8_t F) {
    if ((F & ZERO_FLAG) == 0) {
        RAM[SP] = PC + 3;
        SP++;

        PC++;
        uint16_t adress = RAM[PC];

        PC++;
        adress |= (RAM[PC] << 8);

        PC = adress;
    }
}