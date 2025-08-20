#include <cstdint>

#include "../include/tools.hpp"
#include "../include/const.hpp"

#include "../include/opcodes.hpp"
#include "../include/popcodes.hpp"

#include "../include/instruction.hpp"
#include "../include/prefixed.hpp"

void loadBoot(std::vector<u_int8_t>& RAM, std::string filename) {
    char value;

    std::fstream rom;
    rom.open(filename, std::fstream::in | std::fstream::binary);

    u_int16_t counter = 0x0000;

    while (rom.read(&value, 1) && counter < BOOTSIZE) {
        RAM[counter] = value;
        counter++;
    }
}

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
    for (int i = 0x1; i < ROMSIZE + 2; i++) {
        std::cout << std::hex << int(RAM[i - 1]) << "\t";
        if (i % 0x10 == 0) {
            std::cout << std::endl;
        }
    }

    std::cout << std::endl;
}

void displayMemorySection(std::vector<u_int8_t>& RAM, int start, int end) {
    for (int i = start + 1; i < end + 2; i++) {
        std::cout << std::hex << int(RAM[i - 1]) << "\t";
        if (i % 0x10 == 0) {
            std::cout << std::endl;
        }
    }

    std::cout << std::endl;
}

void flagsOutput(uint8_t F) {
    std::cout << "Z : " << (F >> 7) << " N : " << ((F >> 6) & 0b1) << " H : " << ((F >> 5) & 0b1) << " C : " << ((F >> 4) & 0b1) << std::endl;
}

std::string R16_to_str(uint16_t R) {
    std::string result = "0x";

    for (int i = 12; i >= 0; i -= 4) {
        result += hex[(R >> i) & 0xF];
    }

    return result;
}

std::string R8_to_str(uint16_t R) {
    std::string result = "0x";

    for (int i = 4; i >= 0; i -= 4) {
        result += hex[(R >> i) & 0xF];
    }

    return result;
}

void ECI(uint8_t& A, uint8_t& F, uint8_t& B, uint8_t& C, uint8_t& D, uint8_t& E, uint8_t& H, uint8_t& L, uint16_t& SP, uint16_t& PC, std::vector<uint8_t>& RAM, int& cycles_counter) {
    int complement = 0;

    cycles_counter += instructions_cycles[RAM[PC]];

    switch (RAM[PC]) {
        case NOP_OP:
            NOP(PC);
            break;
        case LD_BC_16_OP:
            LD_R16_16(PC, B, C, RAM);
            break;
        case LD_ADBC_A_OP:
            LD_ADR16_R(PC, RAM, B, C, A);
            break;
        case INC_BC_OP:
            INC_R16(PC, B, C);
            break;
        case INC_B_OP:
            INC_R8(PC, B, F);
            break;
        case DEC_B_OP:
            DEC_R8(PC, B, F);
            break;
        case LD_B_8_OP:
            LD_R8_8(PC, B, RAM);
            break;
        case RLCA_OP:
            RLCA(PC, A, F);
            break;
        case LD_AD16_SP_OP:
            LD_AD16_SP(PC, RAM, SP);
            break;
        case ADD_HL_BC_OP:
            ADD_R16_R16(PC, H, L, B, C, F);
            break;
        case LD_A_ADBC_OP:
            LD_R8_ADR16(PC, A, RAM, B, C);
            break;
        case DEC_BC_OP:
            DEC_R16(PC, B, C);
            break;
        case INC_C_OP:
            INC_R8(PC, C, F);
            break;
        case DEC_C_OP:
            DEC_R8(PC, C, F);
            break;
        case LD_C_8_OP:
            LD_R8_8(PC, C, RAM);
            break;
        case RRCA_OP:
            RRCA(PC, A, F);
            break;
        case STOP_8_OP:
            STOP_8(PC);
            break;
        case LD_DE_16_OP:
            LD_R16_16(PC, D, E, RAM);
            break;
        case LD_ADDE_A_OP:
            LD_ADR16_R(PC, RAM, D, E, A);
            break;
        case INC_DE_OP:
            INC_R16(PC, D, E);
            break;
        case INC_D_OP:
            INC_R8(PC, D, F);
            break;
        case DEC_D_OP:
            DEC_R8(PC, D, F);
            break;
        case LD_D_8_OP:
            LD_R8_8(PC, D, RAM);
            break;
        case RLA_OP:
            RLA(PC, A, F);
            break;
        case JR_8_OP:
            JR_8(PC, RAM);
            break;
        case ADD_HL_DE_OP:
            ADD_R16_R16(PC, H, L, D, E, F);
            break;
        case LD_A_ADDE_OP:
            LD_R8_ADR16(PC, A, RAM, D, E);
            break;
        case DEC_DE_OP:
            DEC_R16(PC, D, E);
            break;
        case INC_E_OP:
            INC_R8(PC, E, F);
            break;
        case DEC_E_OP:
            DEC_R8(PC, E, F);
            break;
        case LD_E_8_OP:
            LD_R8_8(PC, E, RAM);
            break;
        case RRA_OP:
            RRA(PC, A, F);
            break;
        case JR_NZ_8_OP:
            JR_NZ_8(PC, F, RAM);
            if ((F & ZERO_FLAG) == 0) {
                complement = 1;
            }
            break;
        case LD_HL_16_OP:
            LD_R16_16(PC, H, L, RAM);
            break;
        case LD_ADHLP_A_OP:
            LD_ADHL_I_A(PC, RAM, H, L, A);
            break;
        case INC_HL_OP:
            INC_R16(PC, H, L);
            break;
        case INC_H_OP:
            INC_R8(PC, H, F);
            break;
        case DEC_H_OP:
            DEC_R8(PC, H, F);
            break;
        case LD_H_8_OP:
            LD_R8_8(PC, H, RAM);
            break;
        case DAA_OP:
            DAA(PC, A, F);
            break;
        case JR_Z_8_OP:
            JR_Z_8(PC, F, RAM);
            if ((F & ZERO_FLAG) != 0) {
                complement = 1;
            }
            break;
        case ADD_HL_HL_OP:
            ADD_R16_R16(PC, H, L, H, L, F);
            break;
        case LD_A_ADHLP_OP:
            LD_A_ADHL_I(PC, RAM, H, L, A);
            break;
        case DEC_HL_OP:
            DEC_R16(PC, H, L);
            break;
        case INC_L_OP:
            INC_R8(PC, L, F);
            break;
        case DEC_L_OP:
            DEC_R8(PC, L, F);
            break;
        case LD_L_8_OP:
            LD_R8_8(PC, L, RAM);
            break;
        case CPL_OP:
            CPL(PC, A, F);
            break;
        case JR_NC_8_OP:
            JR_NC_8(PC, F, RAM);
            if ((F & CARRY_FLAG) == 0) {
                complement = 1;
            }
            break;
        case LD_SP_16_OP:
            LD_SP_16(PC, SP, RAM);
            break;
        case LD_ADHLM_A_OP:
            LD_ADHL_D_A(PC, RAM, H, L, A);
            break;
        case INC_SP_OP:
            INC_SP(PC, SP);
            break;
        case INC_ADHL_OP:
            INC_ADHL(PC, RAM, H, L, F);
            break;
        case DEC_ADHL_OP:
            DEC_ADHL(PC, RAM, H, L, F);
            break;
        case LD_ADHL_8_OP:
            LD_ADHL_8(PC, RAM, H, L);
            break;
        case SCF_OP:
            SCF(PC, F);
            break;
        case JR_C_8_OP:
            JR_C_8(PC, F, RAM);
            if ((F & CARRY_FLAG) != 0) {
                complement = 1;
            }
            break;
        case ADD_HL_SP_OP:
            ADD_HL_SP(PC, H, L, SP, F);
            break;
        case LD_A_ADHLM_OP:
            LD_A_ADHL_D(PC, RAM, H, L, A);
            break;
        case DEC_SP_OP:
            DEC_SP(PC, SP);
            break;
        case INC_A_OP:
            INC_R8(PC, A, F);
            break;
        case DEC_A_OP:
            DEC_R8(PC, A, F);
            break;
        case LD_A_8_OP:
            LD_R8_8(PC, A, RAM);
            break;
        case CCF_OP:
            CCF(PC, F);
            break;
        case LD_B_B_OP:
            LD_R8_R8(PC, B, B);
            break;
        case LD_B_C_OP:
            LD_R8_R8(PC, B, C);
            break;
        case LD_B_D_OP:
            LD_R8_R8(PC, B, D);
            break;
        case LD_B_E_OP:
            LD_R8_R8(PC, B, E);
            break;
        case LD_B_H_OP:
            LD_R8_R8(PC, B, H);
            break;
        case LD_B_L_OP:
            LD_R8_R8(PC, B, L);
            break;
        case LD_B_ADHL_OP:
            LD_R8_ADR16(PC, B, RAM, H, L);
            break;
        case LD_B_A_OP:
            LD_R8_R8(PC, B, A);
            break;
        case LD_C_B_OP:
            LD_R8_R8(PC, C, B);
            break;
        case LD_C_C_OP:
            LD_R8_R8(PC, C, C);
            break;
        case LD_C_D_OP:
            LD_R8_R8(PC, C, D);
            break;
        case LD_C_E_OP:
            LD_R8_R8(PC, C, E);
            break;
        case LD_C_H_OP:
            LD_R8_R8(PC, C, H);
            break;
        case LD_C_L_OP:
            LD_R8_R8(PC, C, L);
            break;
        case LD_C_ADHL_OP:
            LD_R8_ADR16(PC, C, RAM, H, L);
            break;
        case LD_C_A_OP:
            LD_R8_R8(PC, C, A);
            break;
        case LD_D_B_OP:
            LD_R8_R8(PC, D, B);
            break;
        case LD_D_C_OP:
            LD_R8_R8(PC, D, C);
            break;
        case LD_D_D_OP:
            LD_R8_R8(PC, D, D);
            break;
        case LD_D_E_OP:
            LD_R8_R8(PC, D, E);
            break;
        case LD_D_H_OP:
            LD_R8_R8(PC, D, H);
            break;
        case LD_D_L_OP:
            LD_R8_R8(PC, D, L);
            break;
        case LD_D_ADHL_OP:
            LD_R8_ADR16(PC, D, RAM, H, L);
            break;
        case LD_D_A_OP:
            LD_R8_R8(PC, D, A);
            break;
        case LD_E_B_OP:
            LD_R8_R8(PC, E, B);
            break;
        case LD_E_C_OP:
            LD_R8_R8(PC, E, C);
            break;
        case LD_E_D_OP:
            LD_R8_R8(PC, E, D);
            break;
        case LD_E_E_OP:
            LD_R8_R8(PC, E, E);
            break;
        case LD_E_H_OP:
            LD_R8_R8(PC, E, H);
            break;
        case LD_E_L_OP:
            LD_R8_R8(PC, E, L);
            break;
        case LD_E_ADHL_OP:
            LD_R8_ADR16(PC, E, RAM, H, L);
            break;
        case LD_E_A_OP:
            LD_R8_R8(PC, E, A);
            break;
        case LD_H_B_OP:
            LD_R8_R8(PC, H, B);
            break;
        case LD_H_C_OP:
            LD_R8_R8(PC, H, C);
            break;
        case LD_H_D_OP:
            LD_R8_R8(PC, H, D);
            break;
        case LD_H_E_OP:
            LD_R8_R8(PC, H, E);
            break;
        case LD_H_H_OP:
            LD_R8_R8(PC, H, H);
            break;
        case LD_H_L_OP:
            LD_R8_R8(PC, H, L);
            break;
        case LD_H_ADHL_OP:
            LD_R8_ADR16(PC, H, RAM, H, L);
            break;
        case LD_H_A_OP:
            LD_R8_R8(PC, H, A);
            break;
        case LD_L_B_OP:
            LD_R8_R8(PC, L, B);
            break;
        case LD_L_C_OP:
            LD_R8_R8(PC, L, C);
            break;
        case LD_L_D_OP:
            LD_R8_R8(PC, L, D);
            break;
        case LD_L_E_OP:
            LD_R8_R8(PC, L, E);
            break;
        case LD_L_H_OP:
            LD_R8_R8(PC, L, H);
            break;
        case LD_L_L_OP:
            LD_R8_R8(PC, L, L);
            break;
        case LD_L_ADHL_OP:
            LD_R8_ADR16(PC, L, RAM, H, L);
            break;
        case LD_L_A_OP:
            LD_R8_R8(PC, L, A);
            break;
        case LD_ADHL_B_OP:
            LD_ADHL_R8(PC, RAM, H, L, B);
            break;
        case LD_ADHL_C_OP:
            LD_ADHL_R8(PC, RAM, H, L, C);
            break;
        case LD_ADHL_D_OP:
            LD_ADHL_R8(PC, RAM, H, L, D);
            break;
        case LD_ADHL_E_OP:
            LD_ADHL_R8(PC, RAM, H, L, E);
            break;
        case LD_ADHL_H_OP:
            LD_ADHL_R8(PC, RAM, H, L, H);
            break;
        case LD_ADHL_L_OP:
            LD_ADHL_R8(PC, RAM, H, L, L);
            break;
        case HALT_OP:
            HALT(PC);
            break;
        case LD_ADHL_A_OP:
            LD_ADHL_R8(PC, RAM, H, L, A);
            break;
        case LD_A_B_OP:
            LD_R8_R8(PC, A, B);
            break;
        case LD_A_C_OP:
            LD_R8_R8(PC, A, C);
            break;
        case LD_A_D_OP:
            LD_R8_R8(PC, A, D);
            break;
        case LD_A_E_OP:
            LD_R8_R8(PC, A, E);
            break;
        case LD_A_H_OP:
            LD_R8_R8(PC, A, H);
            break;
        case LD_A_L_OP:
            LD_R8_R8(PC, A, L);
            break;
        case LD_A_ADHL_OP:
            LD_R8_ADR16(PC, A, RAM, H, L);
            break;
        case LD_A_A_OP:
            LD_R8_R8(PC, A, A);
            break;
        case ADD_A_B_OP:
            ADD_A_8(PC, A, RAM, F);
            break;
        case ADD_A_C_OP:
            ADD_R_R(PC, A, C, F);
            break;
        case ADD_A_D_OP:
            ADD_R_R(PC, A, D, F);
            break;
        case ADD_A_E_OP:
            ADD_R_R(PC, A, E, F);
            break;
        case ADD_A_H_OP:
            ADD_R_R(PC, A, H, F);
            break;
        case ADD_A_L_OP:
            ADD_R_R(PC, A, L, F);
            break;
        case ADD_A_ADHL_OP:
            ADD_R_ADHL(PC, A, RAM, H, L, F);
            break;
        case ADD_A_A_OP:
            ADD_R_R(PC, A, A, F);
            break;
        case ADC_A_B_OP:
            ADC_R_R(PC, A, B, F);
            break;
        case ADC_A_C_OP:
            ADC_R_R(PC, A, C, F);
            break;
        case ADC_A_D_OP:
            ADC_R_R(PC, A, D, F);
            break;
        case ADC_A_E_OP:
            ADC_R_R(PC, A, E, F);
            break;
        case ADC_A_H_OP:
            ADC_R_R(PC, A, H, F);
            break;
        case ADC_A_L_OP:
            ADC_R_R(PC, A, L, F);
            break;
        case ADC_A_ADHL_OP:
            ADC_R_ADHL(PC, A, RAM, H, L, F);
            break;
        case ADC_A_A_OP:
            ADC_R_R(PC, A, A, F);
            break;
        case SUB_A_B_OP:
            SUB_R_R(PC, A, B, F);
            break;
        case SUB_A_C_OP:
            SUB_R_R(PC, A, C, F);
            break;
        case SUB_A_D_OP:
            SUB_R_R(PC, A, D, F);
            break;
        case SUB_A_E_OP:
            SUB_R_R(PC, A, E, F);
            break;
        case SUB_A_H_OP:
            SUB_R_R(PC, A, H, F);
            break;
        case SUB_A_L_OP:
            SUB_R_R(PC, A, L, F);
            break;
        case SUB_A_ADHL_OP:
            SUB_R_ADHL(PC, A, RAM, H, L, F);
            break;
        case SUB_A_A_OP:
            SUB_R_R(PC, A, A, F);
            break;
        case SBC_A_B_OP:
            SBC_R_R(PC, A, B, F);
            break;
        case SBC_A_C_OP:
            SBC_R_R(PC, A, C, F);
            break;
        case SBC_A_D_OP:
            SBC_R_R(PC, A, D, F);
            break;
        case SBC_A_E_OP:
            SBC_R_R(PC, A, E, F);
            break;
        case SBC_A_H_OP:
            SBC_R_R(PC, A, H, F);
            break;
        case SBC_A_L_OP:
            SBC_R_R(PC, A, L, F);
            break;
        case SBC_A_ADHL_OP:
            SBC_R_ADHL(PC, A, RAM, H, L, F);
            break;
        case SBC_A_A_OP:
            SBC_R_R(PC, A, A, F);
            break;
        case AND_A_B_OP:
            AND_R_R(PC, A, B, F);
            break;
        case AND_A_C_OP:
            AND_R_R(PC, A, C, F);
            break;
        case AND_A_D_OP:
            AND_R_R(PC, A, D, F);
            break;
        case AND_A_E_OP:
            AND_R_R(PC, A, E, F);
            break;
        case AND_A_H_OP:
            AND_R_R(PC, A, H, F);
            break;
        case AND_A_L_OP:
            AND_R_R(PC, A, L, F);
            break;
        case AND_A_ADHL_OP:
            AND_R_ADHL(PC, A, RAM, H, L, F);
            break;
        case AND_A_A_OP:
            AND_R_R(PC, A, A, F);
            break;
        case XOR_A_B_OP:
            XOR_R_R(PC, A, B, F);
            break;
        case XOR_A_C_OP:
            XOR_R_R(PC, A, C, F);
            break;
        case XOR_A_D_OP:
            XOR_R_R(PC, A, D, F);
            break;
        case XOR_A_E_OP:
            XOR_R_R(PC, A, E, F);
            break;
        case XOR_A_H_OP:
            XOR_R_R(PC, A, H, F);
            break;
        case XOR_A_L_OP:
            XOR_R_R(PC, A, L, F);
            break;
        case XOR_A_ADHL_OP:
            XOR_R_ADHL(PC, A, RAM, H, L, F);
            break;
        case XOR_A_A_OP:
            XOR_R_R(PC, A, A, F);
            break;
        case OR_A_B_OP:
            OR_R_R(PC, A, B, F);
            break;
        case OR_A_C_OP:
            OR_R_R(PC, A, C, F);
            break;
        case OR_A_D_OP:
            OR_R_R(PC, A, D, F);
            break;
        case OR_A_E_OP:
            OR_R_R(PC, A, E, F);
            break;
        case OR_A_H_OP:
            OR_R_R(PC, A, H, F);
            break;
        case OR_A_L_OP:
            OR_R_R(PC, A, L, F);
            break;
        case OR_A_ADHL_OP:
            OR_R_ADHL(PC, A, RAM, H, L, F);
            break;
        case OR_A_A_OP:
            OR_R_R(PC, A, A, F);
            break;
        case CP_A_B_OP:
            CP_R_R(PC, A, B, F);
            break;
        case CP_A_C_OP:
            CP_R_R(PC, A, C, F);
            break;
        case CP_A_D_OP:
            CP_R_R(PC, A, D, F);
            break;
        case CP_A_E_OP:
            CP_R_R(PC, A, E, F);
            break;
        case CP_A_H_OP:
            CP_R_R(PC, A, H, F);
            break;
        case CP_A_L_OP:
            CP_R_R(PC, A, L, F);
            break;
        case CP_A_ADHL_OP:
            CP_R_ADHL(PC, A, RAM, H, L, F);
            break;
        case CP_A_A_OP:
            CP_R_R(PC, A, A, F);
            break;
        case RET_NZ_OP:
            RET_NZ(PC, RAM, SP, F);
            if ((F & ZERO_FLAG) == 0) {
                complement = 3;
            }
            break;
        case POP_BC_OP:
            POP_R16(PC, RAM, SP, B, C);
            break;
        case JP_NZ_16_OP:
            JP_NZ_16(PC, RAM, F);
            if ((F & ZERO_FLAG) == 0) {
                complement = 1;
            }
            break;
        case JP_16_OP:
            JP_16(PC, RAM);
            break;
        case CALL_NZ_16_OP:
            CALL_NZ_16(PC, RAM, SP, F);
            if ((F & ZERO_FLAG) == 0) {
                complement = 3;
            }
            break;
        case PUSH_BC_OP:
            PUSH_R16(PC, RAM, SP, B, C);
            break;
        case ADD_A_8_OP:
            ADD_A_8(PC, A, RAM, F);
            break;
        case RST_00_OP:
            RST_AD(PC, RAM, SP, 0x00);
            break;
        case RET_Z_OP:
            RET_Z(PC, RAM, SP, F);
            if ((F & ZERO_FLAG) != 0) {
                complement = 3;
            }
            break;
        case RET_OP:
            RET(PC, RAM, SP);
            break;
        case JP_Z_16_OP:
            JP_Z_16(PC, RAM, F);
            if ((F & ZERO_FLAG) != 0) {
                complement = 1;
            }
            break;
        case PREFIX_OP:
            PC++;
            EPR(A, F, B, C, D, E, H, L, SP, PC, RAM, cycles_counter);
            break;
        case CALL_Z_16_OP:
            CALL_Z_16(PC, RAM, SP, F);
            if ((F & ZERO_FLAG) != 0) {
                complement = 3;
            }
            break;
        case CALL_16_OP:
            CALL_16(PC, RAM, SP);
            break;
        case ADC_A_8_OP:
            ADC_A_8(PC, A, RAM, F);
            break;
        case RST_08_OP:
            RST_AD(PC, RAM, SP, 0x08);
            break;
        case RET_NC_OP:
            RET_NC(PC, RAM, SP, F);
            if ((F & CARRY_FLAG) == 0) {
                complement = 3;
            }
            break;
        case POP_DE_OP:
            POP_R16(PC, RAM, SP, D, E);
            break;
        case JP_NC_16_OP:
            JP_NC_16(PC, RAM, F);
            if ((F & CARRY_FLAG) == 0) {
                complement = 1;
            }
            break;
        case CALL_NC_16_OP:
            CALL_NC_16(PC, RAM, SP, F);
            if ((F & CARRY_FLAG) == 0) {
                complement = 3;
            }
            break;
        case PUSH_DE_OP:
            PUSH_R16(PC, RAM, SP, D, E);
            break;
        case SUB_A_8_OP:
            SUB_A_8(PC, A, RAM, F);
            break;
        case RST_10_OP:
            RST_AD(PC, RAM, SP, 0x10);
            break;
        case RET_C_OP:
            RET_C(PC, RAM, SP, F);
            if ((F & CARRY_FLAG) != 0) {
                complement = 3;
            }
            break;
        case RETI_OP:
            RETI(PC);
            break;
        case JP_C_16_OP:
            JP_C_16(PC, RAM, F);
            if ((F & CARRY_FLAG) != 0) {
                complement = 1;
            }
            break;
        case CALL_C_16_OP:
            CALL_C_16(PC, RAM, SP, F);
            if ((F & CARRY_FLAG) != 0) {
                complement = 3;
            }
            break;
        case SBC_A_8_OP:
            SBC_A_8(PC, A, RAM, F);
            break;
        case RST_18_OP:
            RST_AD(PC, RAM, SP, 0x18);
            break;
        case LDH_AD8_A_OP:
            LDH_AD8_A(PC, RAM, A);
            break;
        case POP_HL_OP:
            POP_R16(PC, RAM, SP, H, L);
            break;
        case LDH_ADC_A_OP:
            LDH_ADC_A(PC, RAM, C, A);
            break;
        case PUSH_HL_OP:
            PUSH_R16(PC, RAM, SP, H, L);
            break;
        case AND_A_8_OP:
            AND_A_8(PC, A, RAM, F);
            break;
        case RST_20_OP:
            RST_AD(PC, RAM, SP, 0x20);
            break;
        case ADD_SP_8_OP:
            ADD_SP_8(PC, SP, RAM);
            break;
        case JP_HL_OP:
            JP_HL(PC, H, L);
            break;
        case LD_AD16_A_OP:
            LD_AD16_A(PC, RAM, A);
            break;
        case XOR_A_8_OP:
            XOR_A_8(PC, A, RAM, F);
            break;
        case RST_28_OP:
            RST_AD(PC, RAM, SP, 0x28);
            break;
        case LDH_A_AD8_OP:
            LDH_A_AD8(PC, A, RAM);
            break;
        case POP_AF_OP:
            POP_R16(PC, RAM, SP, A, F);
            break;
        case LDH_A_ADC_OP:
            LDH_A_ADC(PC, A, RAM, C);
            break;
        case DI_OP:
            std::cout << "Should execute DI instruction" << std::endl;
            PC++;
            break;
        case PUSH_AF_OP:
            PUSH_R16(PC, RAM, SP, A, F);
            break;
        case OR_A_8_OP:
            OR_A_8(PC, A, RAM, F);
            break;
        case RST_30_OP:
            RST_AD(PC, RAM, SP, 0x30);
            break;
        case LD_HL_SP8_OP:
            LD_HL_SP8(PC, H, L, SP, RAM, F);
            break;
        case LD_SP_HL_OP:
            LD_SP_HL(PC, SP, H, L);
            break;
        case LD_A_AD16_OP:
            LD_A_AD16(PC, A, RAM);
            break;
        case EI_OP:
            std::cout << "Should execute EI instruction" << std::endl;
            PC++;
            break;
        case CP_A_8_OP:
            CP_A_8(PC, A, RAM, F);
            break;
        case RST_38_OP:
            RST_AD(PC, RAM, SP, 0x38);
            break;
        default:
            std::cout << "No correspondence found" << std::endl;
            PC++;
            break;
    }

    cycles_counter += complement;
}

void EPR(uint8_t& A, uint8_t& F, uint8_t& B, uint8_t& C, uint8_t& D, uint8_t& E, uint8_t& H, uint8_t& L, uint16_t& SP, uint16_t& PC, std::vector<uint8_t>& RAM, int& cycles_counter) {

    cycles_counter += prefixed_cycles[RAM[PC]];

    switch (RAM[PC]) {
        case RLC_B_OP:
            RLC_R(PC, B, F);
            break;
        case RLC_C_OP:
            RLC_R(PC, C, F);
            break;
        case RLC_D_OP:
            RLC_R(PC, D, F);
            break;
        case RLC_E_OP:
            RLC_R(PC, E, F);
            break;
        case RLC_H_OP:
            RLC_R(PC, H, F);
            break;
        case RLC_L_OP:
            RLC_R(PC, L, F);
            break;
        case RLC_ADHL_OP:
            RLC_ADHL(PC, RAM, H, L, F);
            break;
        case RLC_A_OP:
            RLC_R(PC, A, F);
            break;
        case RRC_B_OP:
            RRC_R(PC, B, F);
            break;
        case RRC_C_OP:
            RRC_R(PC, C, F);
            break;
        case RRC_D_OP:
            RRC_R(PC, D, F);
            break;
        case RRC_E_OP:
            RRC_R(PC, E, F);
            break;
        case RRC_H_OP:
            RRC_R(PC, H, F);
            break;
        case RRC_L_OP:
            RRC_R(PC, L, F);
            break;
        case RRC_ADHL_OP:
            RRC_ADHL(PC, RAM, H, L, F);
            break;
        case RRC_A_OP:
            RRC_R(PC, A, F);
            break;
        case RL_B_OP:
            RL_R(PC, B, F);
            break;
        case RL_C_OP:
            RL_R(PC, C, F);
            break;
        case RL_D_OP:
            RL_R(PC, D, F);
            break;
        case RL_E_OP:
            RL_R(PC, E, F);
            break;
        case RL_H_OP:
            RL_R(PC, H, F);
            break;
        case RL_L_OP:
            RL_R(PC, L, F);
            break;
        case RL_ADHL_OP:
            RL_ADHL(PC, RAM, H, L, F);
            break;
        case RL_A_OP:
            RL_R(PC, A, F);
            break;
        case RR_B_OP:
            RR_R(PC, B, F);
            break;
        case RR_C_OP:
            RR_R(PC, C, F);
            break;
        case RR_D_OP:
            RR_R(PC, D, F);
            break;
        case RR_E_OP:
            RR_R(PC, E, F);
            break;
        case RR_H_OP:
            RR_R(PC, H, F);
            break;
        case RR_L_OP:
            RR_R(PC, L, F);
            break;
        case RR_ADHL_OP:
            RR_ADHL(PC, RAM, H, L, F);
            break;
        case RR_A_OP:
            RR_R(PC, A, F);
            break;
        case SLA_B_OP:
            SLA_R(PC, B, F);
            break;
        case SLA_C_OP:
            SLA_R(PC, C, F);
            break;
        case SLA_D_OP:
            SLA_R(PC, D, F);
            break;
        case SLA_E_OP:
            SLA_R(PC, E, F);
            break;
        case SLA_H_OP:
            SLA_R(PC, H, F);
            break;
        case SLA_L_OP:
            SLA_R(PC, L, F);
            break;
        case SLA_ADHL_OP:
            SLA_ADHL(PC, RAM, H, L, F);
            break;
        case SLA_A_OP:
            SLA_R(PC, A, F);
            break;
        case SRA_B_OP:
            SRA_R(PC, B, F);
            break;
        case SRA_C_OP:
            SRA_R(PC, C, F);
            break;
        case SRA_D_OP:
            SRA_R(PC, D, F);
            break;
        case SRA_E_OP:
            SRA_R(PC, E, F);
            break;
        case SRA_H_OP:
            SRA_R(PC, H, F);
            break;
        case SRA_L_OP:
            SRA_R(PC, L, F);
            break;
        case SRA_ADHL_OP:
            SRA_ADHL(PC, RAM, H, L, F);
            break;
        case SRA_A_OP:
            SRA_R(PC, A, F);
            break;
        case SWAP_B_OP:
            SWAP_R(PC, B, F);
            break;
        case SWAP_C_OP:
            SWAP_R(PC, C, F);
            break;
        case SWAP_D_OP:
            SWAP_R(PC, D, F);
            break;
        case SWAP_E_OP:
            SWAP_R(PC, E, F);
            break;
        case SWAP_H_OP:
            SWAP_R(PC, H, F);
            break;
        case SWAP_L_OP:
            SWAP_R(PC, L, F);
            break;
        case SWAP_ADHL_OP:
            SWAP_ADHL(PC, RAM, H, L, F);
            break;
        case SWAP_A_OP:
            SWAP_R(PC, A, F);
            break;
         case SRL_B_OP:
            SRL_R(PC, B, F);
            break;
        case SRL_C_OP:
            SRL_R(PC, C, F);
            break;
        case SRL_D_OP:
            SRL_R(PC, D, F);
            break;
        case SRL_E_OP:
            SRL_R(PC, E, F);
            break;
        case SRL_H_OP:
            SRL_R(PC, H, F);
            break;
        case SRL_L_OP:
            SRL_R(PC, L, F);
            break;
        case SRL_ADHL_OP:
            SRL_ADHL(PC, RAM, H, L, F);
            break;
        case SRL_A_OP:
            SRL_R(PC, A, F);
            break;
        case BIT_0_B_OP:
            BIT_N_R(PC, B, F, 0);
            break;
        case BIT_0_C_OP:
            BIT_N_R(PC, C, F, 0);
            break;
        case BIT_0_D_OP:
            BIT_N_R(PC, D, F, 0);
            break;
        case BIT_0_E_OP:
            BIT_N_R(PC, E, F, 0);
            break;
        case BIT_0_H_OP:
            BIT_N_R(PC, H, F, 0);
            break;
        case BIT_0_L_OP:
            BIT_N_R(PC, L, F, 0);
            break;
        case BIT_0_ADHL_OP:
            BIT_N_ADHL(PC, RAM, H, L, F, 0);
            break;
        case BIT_0_A_OP:
            BIT_N_R(PC, A, F, 0);
            break;
        case BIT_1_B_OP:
            BIT_N_R(PC, B, F, 1);
            break;
        case BIT_1_C_OP:
            BIT_N_R(PC, C, F, 1);
            break;
        case BIT_1_D_OP:
            BIT_N_R(PC, D, F, 1);
            break;
        case BIT_1_E_OP:
            BIT_N_R(PC, E, F, 1);
            break;
        case BIT_1_H_OP:
            BIT_N_R(PC, H, F, 1);
            break;
        case BIT_1_L_OP:
            BIT_N_R(PC, L, F, 1);
            break;
        case BIT_1_ADHL_OP:
            BIT_N_ADHL(PC, RAM, H, L, F, 1);
            break;
        case BIT_1_A_OP:
            BIT_N_R(PC, A, F, 1);
            break;
        case BIT_2_B_OP:
            BIT_N_R(PC, B, F, 2);
            break;
        case BIT_2_C_OP:
            BIT_N_R(PC, C, F, 2);
            break;
        case BIT_2_D_OP:
            BIT_N_R(PC, D, F, 2);
            break;
        case BIT_2_E_OP:
            BIT_N_R(PC, E, F, 2);
            break;
        case BIT_2_H_OP:
            BIT_N_R(PC, H, F, 2);
            break;
        case BIT_2_L_OP:
            BIT_N_R(PC, L, F, 2);
            break;
        case BIT_2_ADHL_OP:
            BIT_N_ADHL(PC, RAM, H, L, F, 2);
            break;
        case BIT_2_A_OP:
            BIT_N_R(PC, A, F, 2);
            break;
        case BIT_3_B_OP:
            BIT_N_R(PC, B, F, 3);
            break;
        case BIT_3_C_OP:
            BIT_N_R(PC, C, F, 3);
            break;
        case BIT_3_D_OP:
            BIT_N_R(PC, D, F, 3);
            break;
        case BIT_3_E_OP:
            BIT_N_R(PC, E, F, 3);
            break;
        case BIT_3_H_OP:
            BIT_N_R(PC, H, F, 3);
            break;
        case BIT_3_L_OP:
            BIT_N_R(PC, L, F, 3);
            break;
        case BIT_3_ADHL_OP:
            BIT_N_ADHL(PC, RAM, H, L, F, 3);
            break;
        case BIT_3_A_OP:
            BIT_N_R(PC, A, F, 3);
            break;
        case BIT_4_B_OP:
            BIT_N_R(PC, B, F, 4);
            break;
        case BIT_4_C_OP:
            BIT_N_R(PC, C, F, 4);
            break;
        case BIT_4_D_OP:
            BIT_N_R(PC, D, F, 4);
            break;
        case BIT_4_E_OP:
            BIT_N_R(PC, E, F, 4);
            break;
        case BIT_4_H_OP:
            BIT_N_R(PC, H, F, 4);
            break;
        case BIT_4_L_OP:
            BIT_N_R(PC, L, F, 4);
            break;
        case BIT_4_ADHL_OP:
            BIT_N_ADHL(PC, RAM, H, L, F, 4);
            break;
        case BIT_4_A_OP:
            BIT_N_R(PC, A, F, 4);
            break;
        case BIT_5_B_OP:
            BIT_N_R(PC, B, F, 5);
            break;
        case BIT_5_C_OP:
            BIT_N_R(PC, C, F, 5);
            break;
        case BIT_5_D_OP:
            BIT_N_R(PC, D, F, 5);
            break;
        case BIT_5_E_OP:
            BIT_N_R(PC, E, F, 5);
            break;
        case BIT_5_H_OP:
            BIT_N_R(PC, H, F, 5);
            break;
        case BIT_5_L_OP:
            BIT_N_R(PC, L, F, 5);
            break;
        case BIT_5_ADHL_OP:
            BIT_N_ADHL(PC, RAM, H, L, F, 5);
            break;
        case BIT_5_A_OP:
            BIT_N_R(PC, A, F, 5);
            break;
        case BIT_6_B_OP:
            BIT_N_R(PC, B, F, 6);
            break;
        case BIT_6_C_OP:
            BIT_N_R(PC, C, F, 6);
            break;
        case BIT_6_D_OP:
            BIT_N_R(PC, D, F, 6);
            break;
        case BIT_6_E_OP:
            BIT_N_R(PC, E, F, 6);
            break;
        case BIT_6_H_OP:
            BIT_N_R(PC, H, F, 6);
            break;
        case BIT_6_L_OP:
            BIT_N_R(PC, L, F, 6);
            break;
        case BIT_6_ADHL_OP:
            BIT_N_ADHL(PC, RAM, H, L, F, 6);
            break;
        case BIT_6_A_OP:
            BIT_N_R(PC, A, F, 6);
            break;
        case BIT_7_B_OP:
            BIT_N_R(PC, B, F, 7);
            break;
        case BIT_7_C_OP:
            BIT_N_R(PC, C, F, 7);
            break;
        case BIT_7_D_OP:
            BIT_N_R(PC, D, F, 7);
            break;
        case BIT_7_E_OP:
            BIT_N_R(PC, E, F, 7);
            break;
        case BIT_7_H_OP:
            BIT_N_R(PC, H, F, 7);
            break;
        case BIT_7_L_OP:
            BIT_N_R(PC, L, F, 7);
            break;
        case BIT_7_ADHL_OP:
            BIT_N_ADHL(PC, RAM, H, L, F, 7);
            break;
        case BIT_7_A_OP:
            BIT_N_R(PC, A, F, 7);
            break;
        case RES_0_B_OP:
            RES_N_R(PC, B, 0);
            break;
        case RES_0_C_OP:
            RES_N_R(PC, C, 0);
            break;
        case RES_0_D_OP:
            RES_N_R(PC, D, 0);
            break;
        case RES_0_E_OP:
            RES_N_R(PC, E, 0);
            break;
        case RES_0_H_OP:
            RES_N_R(PC, H, 0);
            break;
        case RES_0_L_OP:
            RES_N_R(PC, L, 0);
            break;
        case RES_0_ADHL_OP:
            RES_N_ADHL(PC, RAM, H, L, 0);
            break;
        case RES_0_A_OP:
            RES_N_R(PC, A, 0);
            break;
        case RES_1_B_OP:
            RES_N_R(PC, B, 1);
            break;
        case RES_1_C_OP:
            RES_N_R(PC, C, 1);
            break;
        case RES_1_D_OP:
            RES_N_R(PC, D, 1);
            break;
        case RES_1_E_OP:
            RES_N_R(PC, E, 1);
            break;
        case RES_1_H_OP:
            RES_N_R(PC, H, 1);
            break;
        case RES_1_L_OP:
            RES_N_R(PC, L, 1);
            break;
        case RES_1_ADHL_OP:
            RES_N_ADHL(PC, RAM, H, L, 1);
            break;
        case RES_1_A_OP:
            RES_N_R(PC, A, 1);
            break;
        case RES_2_B_OP:
            RES_N_R(PC, B, 2);
            break;
        case RES_2_C_OP:
            RES_N_R(PC, C, 2);
            break;
        case RES_2_D_OP:
            RES_N_R(PC, D, 2);
            break;
        case RES_2_E_OP:
            RES_N_R(PC, E, 2);
            break;
        case RES_2_H_OP:
            RES_N_R(PC, H, 2);
            break;
        case RES_2_L_OP:
            RES_N_R(PC, L, 2);
            break;
        case RES_2_ADHL_OP:
            RES_N_ADHL(PC, RAM, H, L, 2);
            break;
        case RES_2_A_OP:
            RES_N_R(PC, A, 2);
            break;
        case RES_3_B_OP:
            RES_N_R(PC, B, 3);
            break;
        case RES_3_C_OP:
            RES_N_R(PC, C, 3);
            break;
        case RES_3_D_OP:
            RES_N_R(PC, D, 3);
            break;
        case RES_3_E_OP:
            RES_N_R(PC, E, 3);
            break;
        case RES_3_H_OP:
            RES_N_R(PC, H, 3);
            break;
        case RES_3_L_OP:
            RES_N_R(PC, L, 3);
            break;
        case RES_3_ADHL_OP:
            RES_N_ADHL(PC, RAM, H, L, 3);
            break;
        case RES_3_A_OP:
            RES_N_R(PC, A, 3);
            break;
        case RES_4_B_OP:
            RES_N_R(PC, B, 4);
            break;
        case RES_4_C_OP:
            RES_N_R(PC, C, 4);
            break;
        case RES_4_D_OP:
            RES_N_R(PC, D, 4);
            break;
        case RES_4_E_OP:
            RES_N_R(PC, E, 4);
            break;
        case RES_4_H_OP:
            RES_N_R(PC, H, 4);
            break;
        case RES_4_L_OP:
            RES_N_R(PC, L, 4);
            break;
        case RES_4_ADHL_OP:
            RES_N_ADHL(PC, RAM, H, L, 4);
            break;
        case RES_4_A_OP:
            RES_N_R(PC, A, 4);
            break;
        case RES_5_B_OP:
            RES_N_R(PC, B, 5);
            break;
        case RES_5_C_OP:
            RES_N_R(PC, C, 5);
            break;
        case RES_5_D_OP:
            RES_N_R(PC, D, 5);
            break;
        case RES_5_E_OP:
            RES_N_R(PC, E, 5);
            break;
        case RES_5_H_OP:
            RES_N_R(PC, H, 5);
            break;
        case RES_5_L_OP:
            RES_N_R(PC, L, 5);
            break;
        case RES_5_ADHL_OP:
            RES_N_ADHL(PC, RAM, H, L, 5);
            break;
        case RES_5_A_OP:
            RES_N_R(PC, A, 5);
            break;
        case RES_6_B_OP:
            RES_N_R(PC, B, 6);
            break;
        case RES_6_C_OP:
            RES_N_R(PC, C, 6);
            break;
        case RES_6_D_OP:
            RES_N_R(PC, D, 6);
            break;
        case RES_6_E_OP:
            RES_N_R(PC, E, 6);
            break;
        case RES_6_H_OP:
            RES_N_R(PC, H, 6);
            break;
        case RES_6_L_OP:
            RES_N_R(PC, L, 6);
            break;
        case RES_6_ADHL_OP:
            RES_N_ADHL(PC, RAM, H, L, 6);
            break;
        case RES_6_A_OP:
            RES_N_R(PC, A, 6);
            break;
        case RES_7_B_OP:
            RES_N_R(PC, B, 7);
            break;
        case RES_7_C_OP:
            RES_N_R(PC, C, 7);
            break;
        case RES_7_D_OP:
            RES_N_R(PC, D, 7);
            break;
        case RES_7_E_OP:
            RES_N_R(PC, E, 7);
            break;
        case RES_7_H_OP:
            RES_N_R(PC, H, 7);
            break;
        case RES_7_L_OP:
            RES_N_R(PC, L, 7);
            break;
        case RES_7_ADHL_OP:
            RES_N_ADHL(PC, RAM, H, L, 7);
            break;
        case RES_7_A_OP:
            RES_N_R(PC, A, 7);
            break;
        case SET_0_B_OP:
            SET_N_R(PC, B, 0);
            break;
        case SET_0_C_OP:
            SET_N_R(PC, C, 0);
            break;
        case SET_0_D_OP:
            SET_N_R(PC, D, 0);
            break;
        case SET_0_E_OP:
            SET_N_R(PC, E, 0);
            break;
        case SET_0_H_OP:
            SET_N_R(PC, H, 0);
            break;
        case SET_0_L_OP:
            SET_N_R(PC, L, 0);
            break;
        case SET_0_ADHL_OP:
            SET_N_ADHL(PC, RAM, H, L, 0);
            break;
        case SET_0_A_OP:
            SET_N_R(PC, A, 0);
            break;
        case SET_1_B_OP:
            SET_N_R(PC, B, 1);
            break;
        case SET_1_C_OP:
            SET_N_R(PC, C, 1);
            break;
        case SET_1_D_OP:
            SET_N_R(PC, D, 1);
            break;
        case SET_1_E_OP:
            SET_N_R(PC, E, 1);
            break;
        case SET_1_H_OP:
            SET_N_R(PC, H, 1);
            break;
        case SET_1_L_OP:
            SET_N_R(PC, L, 1);
            break;
        case SET_1_ADHL_OP:
            SET_N_ADHL(PC, RAM, H, L, 1);
            break;
        case SET_1_A_OP:
            SET_N_R(PC, A, 1);
            break;
        case SET_2_B_OP:
            SET_N_R(PC, B, 2);
            break;
        case SET_2_C_OP:
            SET_N_R(PC, C, 2);
            break;
        case SET_2_D_OP:
            SET_N_R(PC, D, 2);
            break;
        case SET_2_E_OP:
            SET_N_R(PC, E, 2);
            break;
        case SET_2_H_OP:
            SET_N_R(PC, H, 2);
            break;
        case SET_2_L_OP:
            SET_N_R(PC, L, 2);
            break;
        case SET_2_ADHL_OP:
            SET_N_ADHL(PC, RAM, H, L, 2);
            break;
        case SET_2_A_OP:
            SET_N_R(PC, A, 2);
            break;
        case SET_3_B_OP:
            SET_N_R(PC, B, 3);
            break;
        case SET_3_C_OP:
            SET_N_R(PC, C, 3);
            break;
        case SET_3_D_OP:
            SET_N_R(PC, D, 3);
            break;
        case SET_3_E_OP:
            SET_N_R(PC, E, 3);
            break;
        case SET_3_H_OP:
            SET_N_R(PC, H, 3);
            break;
        case SET_3_L_OP:
            SET_N_R(PC, L, 3);
            break;
        case SET_3_ADHL_OP:
            SET_N_ADHL(PC, RAM, H, L, 3);
            break;
        case SET_3_A_OP:
            SET_N_R(PC, A, 3);
            break;
        case SET_4_B_OP:
            SET_N_R(PC, B, 4);
            break;
        case SET_4_C_OP:
            SET_N_R(PC, C, 4);
            break;
        case SET_4_D_OP:
            SET_N_R(PC, D, 4);
            break;
        case SET_4_E_OP:
            SET_N_R(PC, E, 4);
            break;
        case SET_4_H_OP:
            SET_N_R(PC, H, 4);
            break;
        case SET_4_L_OP:
            SET_N_R(PC, L, 4);
            break;
        case SET_4_ADHL_OP:
            SET_N_ADHL(PC, RAM, H, L, 4);
            break;
        case SET_4_A_OP:
            SET_N_R(PC, A, 4);
            break;
        case SET_5_B_OP:
            SET_N_R(PC, B, 5);
            break;
        case SET_5_C_OP:
            SET_N_R(PC, C, 5);
            break;
        case SET_5_D_OP:
            SET_N_R(PC, D, 5);
            break;
        case SET_5_E_OP:
            SET_N_R(PC, E, 5);
            break;
        case SET_5_H_OP:
            SET_N_R(PC, H, 5);
            break;
        case SET_5_L_OP:
            SET_N_R(PC, L, 5);
            break;
        case SET_5_ADHL_OP:
            SET_N_ADHL(PC, RAM, H, L, 5);
            break;
        case SET_5_A_OP:
            SET_N_R(PC, A, 5);
            break;
        case SET_6_B_OP:
            SET_N_R(PC, B, 6);
            break;
        case SET_6_C_OP:
            SET_N_R(PC, C, 6);
            break;
        case SET_6_D_OP:
            SET_N_R(PC, D, 6);
            break;
        case SET_6_E_OP:
            SET_N_R(PC, E, 6);
            break;
        case SET_6_H_OP:
            SET_N_R(PC, H, 6);
            break;
        case SET_6_L_OP:
            SET_N_R(PC, L, 6);
            break;
        case SET_6_ADHL_OP:
            SET_N_ADHL(PC, RAM, H, L, 6);
            break;
        case SET_6_A_OP:
            SET_N_R(PC, A, 6);
            break;
        case SET_7_B_OP:
            SET_N_R(PC, B, 7);
            break;
        case SET_7_C_OP:
            SET_N_R(PC, C, 7);
            break;
        case SET_7_D_OP:
            SET_N_R(PC, D, 7);
            break;
        case SET_7_E_OP:
            SET_N_R(PC, E, 7);
            break;
        case SET_7_H_OP:
            SET_N_R(PC, H, 7);
            break;
        case SET_7_L_OP:
            SET_N_R(PC, L, 7);
            break;
        case SET_7_ADHL_OP:
            SET_N_ADHL(PC, RAM, H, L, 7);
            break;
        case SET_7_A_OP:
            SET_N_R(PC, A, 7);
            break;
    }

}