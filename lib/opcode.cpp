#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "../include/opcode.hpp"

void NOP(u_int16_t& PC) {
    std::cout << "Should sleep during one M-cycle" << std::endl;
    PC++;
}

void LD_BC_16(u_int16_t& PC, u_int8_t& B, u_int8_t& C, std::vector<u_int8_t>& RAM) {
    PC++;
    C = RAM[PC]; // C is loaded first because of low-endian memory
    PC++;
    B = RAM[PC];

    PC++;
}

void LD_ADBC_A(u_int16_t& PC, std::vector<u_int8_t>& RAM, u_int8_t& B, u_int8_t& C, u_int8_t A) {
    u_int16_t combination = (B << 8) | C;
    RAM[combination] = A;

    PC++;
}

void INC_BC(u_int16_t& PC, u_int8_t& B, u_int8_t& C) {
    u_int16_t combination = (B << 8) | C;
    combination++;

    B = combination >> 8;
    C = combination & 0xFF;

    PC++;
}

void INC_B(u_int16_t& PC, u_int8_t& B, u_int8_t& F) {
    B++;

    F &= 0b10111111; // Unset negative flag

    if (B == 0) {
        F |= 0b10000000; // Set zero flag
    }

    // If an half-carry occured, set half-carry flag
    if ((B & 0b00010000) == 0b00010000) {
        F |= 0b00100000;
    }

    PC++;
}

void DEC_B(u_int16_t& PC, u_int8_t& B, u_int8_t& F) {
    B--;

    F |= 0b01000000; // Set negative flag

    if (B == 0) {
        F |= 0b10000000; // Set zero flag
    }

    // If an half-carry occured, set half-carry flag
    // Not sure on this one, a bug could come from this
    if ((B & 0b00001111) == 0b00001111) {
        F |= 0b00100000;
    }

    PC++;
}

void LD_B_8(u_int16_t& PC, u_int8_t& B, std::vector<u_int8_t>& RAM) {
    PC++;
    B = RAM[PC];

    PC++;
}

// The entire function is not optimized
// A much better way should be found
void RLCA(u_int16_t& PC, u_int8_t& A, u_int8_t& F) {
    F &= 0b00011111; // Unset zero negative and half-carry flags

    if ((A & 0b10000000) != 0) {
        F |= 0b00010000; // Set carry flag
        A = (A << 1) | 0xFF;
        A |= 0b00000001;
    } else {
        F &= 0b11101111;
        A = (A << 1) | 0xFF;
    }

    PC ++;
}

void LD_AD16_SP(u_int16_t& PC, std::vector<u_int8_t>& RAM, u_int16_t SP) {
    PC++;
    u_int16_t combination = RAM[PC];

    PC++;
    combination = (combination << 8) | RAM[PC];

    RAM[combination] = SP;

    PC++;
}

void ADD_HL_BC(u_int16_t& PC, u_int8_t& H, u_int8_t& L, u_int8_t B, u_int8_t C, u_int8_t& F) {
    F &= 0b10111111; // Unset negative flag

    u_int16_t HLCombination = (H << 8) | L;
    u_int16_t BCCombination = (B << 8) | C;

    u_int16_t HLBuffer = HLCombination;

    HLCombination += BCCombination;

    if (HLCombination < HLBuffer) {
        F |= 0b00010000; // Set carry flag
    }

    if ((HLCombination & 0x1000) != 0) {
        F |= 0b00100000; // Set half carry flag
    }

    H = HLCombination >> 8;
    L = HLCombination & 0xFF;

    PC++;
}

void LD_A_ADBC(u_int16_t& PC, u_int8_t& A, std::vector<u_int8_t>& RAM, u_int8_t B, u_int8_t C) {
    u_int16_t combination = (B << 8) | C;
    A = RAM[combination];

    PC++;
}

void DEC_BC(u_int16_t& PC, u_int8_t& B, u_int8_t& C) {
    u_int16_t combination = (B << 8) | C;
    combination--;

    B = combination >> 8;
    C = combination & 0xFF;
}

void INC_C(u_int16_t& PC, u_int8_t& C, u_int8_t& F) {
    C++;

    F &= 0b10111111; // Unset negative flag

    if (C == 0) {
        F |= 0b10000000; // Set zero flag
    }

    // If an half-carry occured, set half-carry flag
    if ((C & 0b00010000) == 0b00010000) {
        F |= 0b00100000;
    }

    PC++;
}

void DEC_C(u_int16_t& PC, u_int8_t& C, u_int8_t& F) {
    C--;

    F |= 0b01000000; // Set negative flag

    if (C == 0) {
        F |= 0b10000000; // Set zero flag
    }

    // If an half-carry occured, set half-carry flag
    // Not sure on this one, a bug could come from this
    if ((C & 0b00001111) == 0b00001111) {
        F |= 0b00100000;
    }

    PC++;
}

void LD_C_8(u_int16_t& PC, u_int8_t& C, std::vector<u_int8_t>& RAM) {
    PC++;
    C = RAM[PC];

    PC++;
}

void RRCA(u_int16_t& PC, u_int8_t& A, u_int8_t& F) {
    F &= 0b00011111; // Unset zero negative and half-carry flags

    if ((A & 0b00000001) != 0) {
        F |= 0b00010000; // Set carry flag
        A = A >> 1;
        A |= 0b10000000;
    } else {
        F &= 0b11101111;
        A = A >> 1;
    }

    PC++;
}

void STOP_8(u_int16_t& PC) {
    std::cout << "System should go in low power mod" << std::endl;

    PC += 2;
}

void LD_DE_16(u_int16_t& PC, u_int8_t& D, u_int8_t& E, std::vector<u_int8_t>& RAM) {
    PC++;
    E = RAM[PC]; // C is loaded first because of low-endian memory
    PC++;
    D = RAM[PC];

    PC++;
}

void LD_ADDE_A(u_int16_t& PC, std::vector<u_int8_t>& RAM, u_int8_t D, u_int8_t E, u_int8_t A) {
    u_int16_t combination = (D << 8) | E;
    RAM[combination] = A;

    PC++;
}

void INC_DE(u_int16_t& PC, u_int8_t& D, u_int8_t& E) {
    u_int16_t combination = (D << 8) | E;
    combination++;

    D = combination >> 8;
    E = combination & 0xFF;
}

void INC_D(u_int16_t& PC, u_int8_t& D, u_int8_t& F) {
    F &= 0b10111111; // Unset negative flag
    
    D++;

    if (D == 0) {
        F |= 0b10000000; // Set zero flag
    }

    // If an half-carry occured, set half-carry flag
    if ((D & 0b00010000) == 0b00010000) {
        F |= 0b00100000;
    }

    PC++;
}

void DEC_D(u_int16_t& PC, u_int8_t& D, u_int8_t& F) {
    D--;

    F |= 0b01000000; // Set negative flag

    if (D == 0) {
        F |= 0b10000000; // Set zero flag
    }

    // If an half-carry occured, set half-carry flag
    // Not sure on this one, a bug could come from this
    if ((D & 0b00001111) == 0b00001111) {
        F |= 0b00100000;
    }

    PC++;
}

void LD_D_8(u_int16_t& PC, u_int8_t& D, std::vector<u_int8_t>& RAM) {
    PC++;
    D = RAM[PC];

    PC++;
}

void RLA(u_int16_t& PC, u_int8_t& A, u_int8_t& F) {
    F &= 0b00011111; // Unset zero negative and half-carry flags

    bool mst = false;

    if ((A & 0b10000000) != 0) {
        mst = true;
    }
    
    A = (A << 1) & 0xFF;
    
    // If carry flag is set
    if ((F & 0b00010000) != 0) {
        A |= 0b00000001;
    }

    // Put MST of A into carry
    if (mst) {
        F |= 0b00010000;
    } else {
        F &= 0b11101111;
    }

    PC++;
}

void JR_8(u_int16_t& PC, std::vector<u_int8_t>& RAM) {
    PC++;
    u_int8_t value = RAM[PC];

    PC++;
    PC += (signed char)value;
}

void ADD_HL_DE(u_int16_t& PC, u_int8_t& H, u_int8_t& L, u_int8_t D, u_int8_t E, u_int8_t& F) {
    F &= 0b10111111; // Unset negative flag

    u_int16_t HLCombination = (H << 8) | L;
    u_int16_t DECombination = (D << 8) | E;

    u_int16_t HLBuffer = HLCombination;

    HLCombination += DECombination;

    if (HLCombination < HLBuffer) {
        F |= 0b00010000; // Set carry flag
    }

    if ((HLCombination & 0x1000) != 0) {
        F |= 0b00100000; // Set half carry flag
    }

    H = HLCombination >> 8;
    L = HLCombination & 0xFF;

    PC++;
}

void LD_A_ADDE(u_int16_t& PC, u_int8_t& A, std::vector<u_int8_t>& RAM, u_int8_t D, u_int8_t E) {
    u_int16_t combination = (D << 8) | E;
    A = RAM[combination];

    PC++;
}

void DEC_DE(u_int16_t& PC, u_int8_t& D, u_int8_t& E) {
    u_int16_t combination = (D << 8) | E;
    combination--;

    D = combination >> 8;
    E = combination & 0xFF;

    PC++;
}

void INC_E(u_int16_t& PC, u_int8_t& E, u_int8_t& F) {
    E++;

    F &= 0b10111111; // Unset negative flag

    if (E == 0) {
        F |= 0b10000000; // Set zero flag
    }

    // If an half-carry occured, set half-carry flag
    if ((E & 0b00010000) == 0b00010000) {
        F |= 0b00100000;
    }

    PC++;
}

void DEC_E(u_int16_t& PC, u_int8_t& E, u_int8_t& F) {
    E--;

    F |= 0b01000000; // Set negative flag

    if (E == 0) {
        F |= 0b10000000; // Set zero flag
    }

    // If an half-carry occured, set half-carry flag
    // Not sure on this one, a bug could come from this
    if ((E & 0b00001111) == 0b00001111) {
        F |= 0b00100000;
    }

    PC++;
}

void LD_E_8(u_int16_t& PC, u_int8_t& E, std::vector<u_int8_t>& RAM) {
    PC++;
    E = RAM[PC];

    PC++;
}

void RRA(u_int16_t& PC, u_int8_t& A, u_int8_t& F) {
    F &= 0b00011111; // Unset Zero negative and half-carry flags

    bool lst = false;

    if ((A & 0b00000001) != 0) {
        lst = true;
    }

    A = (A >> 1) & 0xFF;

    // If carry flag is set
    if ((F & 0b00010000) != 0) {
        A |= 0b10000000;
    }

    // Put MST of A into carry
    if (lst) {
        F |= 0b00010000;
    } else {
        F &= 0b11101111;
    }

    PC++;
}

// Stand for Jump If not Zero
void JR_NZ_8(u_int16_t& PC, u_int8_t& F, std::vector<u_int8_t>& RAM) {
    PC++;
    u_int8_t value = RAM[PC];

    PC++;

    if ((F & 0b10000000) != 0) {
        PC += (signed char)value;
    }
}

void LD_HL_16(u_int16_t& PC, u_int8_t& H, u_int8_t& L, std::vector<u_int8_t>& RAM) {
    PC++;
    L = RAM[PC];
    PC++;
    H = RAM[PC];

    PC++;
}

void LD_ADHL_I_A(u_int16_t& PC, std::vector<u_int8_t>& RAM, u_int8_t H, u_int8_t L, u_int8_t A) {
    u_int16_t HLCombination = (H << 8) | L;
    RAM[HLCombination] = A;

    HLCombination++;
    H = HLCombination >> 8;
    L = HLCombination & 0xFF;

    PC++;
}

void INC_HL(u_int16_t& PC, u_int8_t& H, u_int8_t& L) {
    u_int16_t HLCombination = (H << 8) | L;

    HLCombination++;
    H = HLCombination >> 8;
    L = HLCombination & 0xFF;

    PC++;
}

void INC_H(u_int16_t& PC, u_int8_t& H, u_int8_t& F) {
    F &= 0b10111111; // Unset negative flag

    // Check if an half-carry will happen
    if ((H & 0xF) == 0xF) {
        F |= 0b00100000;
    }

    H++;

    if (H == 0) {
        F |= 0b10000000; // Set zero flag
    }

    PC++;
}