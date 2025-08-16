#include <cstdint>

#include "../include/tools.hpp"
#include "../include/const.hpp"
#include "../include/opcodes.hpp"
#include "../include/instruction.hpp"

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

void flagsOutput(uint8_t F) {
    std::cout << "Z : " << (F >> 7) << " N : " << ((F >> 6) & 0b1) << " H : " << ((F >> 5) & 0b1) << " C : " << ((F >> 4) & 0b1) << std::endl;
}

void ECI(uint8_t& A, uint8_t& F, uint8_t& B, uint8_t& C, uint8_t& D, uint8_t& E, uint8_t& H, uint8_t& L, uint16_t& SP, uint16_t& PC, std::vector<uint8_t>& RAM, int& cycles_counter) {
    int complement = 0;

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
            std::cout << "Next instruction is prefixed" << std::endl;
            PC += 2;
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

    cycles_counter += instructions_cycles[RAM[PC]] + complement;
}

