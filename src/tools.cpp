#include <unistd.h>
#include <cstdint>
#include <raylib.h>

#include "../include/tools.hpp"
#include "../include/const.hpp"

#include "../include/opcodes.hpp"
#include "../include/popcodes.hpp"

#include "../include/instruction.hpp"
#include "../include/prefixed.hpp"

#include "../include/rendering.hpp"

void loadBoot(std::vector<u_int8_t>& RAM, std::string filename) {
    char value;

    std::fstream rom;
    rom.open(filename, std::fstream::in | std::fstream::binary);

    u_int16_t counter = 0x0000;

    while (rom.read(&value, 1) && counter <= BOOTSIZE) {
        RAM[counter] = value;
        counter++;
    }
}

void loadROM(std::vector<u_int8_t>& RAM, std::string filename) {
    char value;

    std::fstream rom;
    rom.open(filename, std::fstream::in | std::fstream::binary);

    u_int16_t counter = 0x0000;

    while (rom.read(&value, 1) && counter <= ROMSIZE) {
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
    std::cout << "Displaying ram from 0x" << std::hex << +start << " to 0x" << +end << std::endl;
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

void CALL_Vector(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint16_t vector) {
    uint16_t returnAddress = PC;
    SP--;
    RAM[SP] = returnAddress >> 8;
    SP--;
    RAM[SP] = returnAddress & 0xFF;

    PC = vector;
}

void DMACopy(std::vector<uint8_t>& RAM) {
    for (int i = 0; i < 160; i++) {
        RAM[OAM_START + i] = RAM[(RAM[DMA] << 8) + i];
    }
}

void writeRAM(uint16_t address, uint8_t value, std::vector<uint8_t>& RAM) {
    if (address == JOYP) {{}
        if (value >= 0x30) {
            RAM[address] = value | 0x0F;
        } else if (value == 0x20) {
            uint8_t dpad = 0x0F;

            if (IsKeyDown(KEY_UP)) {
                dpad &= ~0x04;
            } else if (IsKeyUp(KEY_UP)) {
                dpad |= 0x04;
            }

            if (IsKeyDown(KEY_DOWN)) {
                dpad &= ~0x08;
            } else if (IsKeyUp(KEY_DOWN)){
                dpad |= 0x08;
            }
            
            if (IsKeyDown(KEY_LEFT)) {
                dpad &= ~0x02;
            } else if (IsKeyUp(KEY_LEFT)) {
                dpad |= 0x02;
            }

            if (IsKeyDown(KEY_RIGHT)) {
                dpad &= ~0x01;
            } else if (IsKeyUp(KEY_RIGHT)) {
                dpad |= 0x01;
            }

            RAM[address] = (value & 0xF0) | dpad;
        } else {
            uint8_t buttons = 0x0F;

            if (IsKeyDown(KEY_E)) {
                buttons &= ~0x04;
            } else if (IsKeyUp(KEY_E)) {
                buttons |= 0x04;
            }

            if (IsKeyDown(KEY_R)) {
                buttons &= ~0x08;
            } else if (IsKeyUp(KEY_R)){
                buttons |= 0x08;
            }
            
            if (IsKeyDown(KEY_Z)) {
                buttons &= ~0x02;
            } else if (IsKeyUp(KEY_Z)) {
                buttons |= 0x02;
            }

            if (IsKeyDown(KEY_A)) {
                buttons &= ~0x01;
            } else if (IsKeyUp(KEY_A)) {
                buttons |= 0x01;
            }

            RAM[address] = (value & 0xF0) | buttons;
        }
    } else {
        RAM[address] = value;
    }
    
    if (address == DMA) {
        DMACopy(RAM);
    }
}

void doCPUStuff(Registers& registers, std::vector<uint8_t>& RAM, Image& screen) {
    ECI(registers, RAM);
            
    if ((RAM[LCDC] & 0x80) == 0x80) {
        if (registers.cycles_counter >= 114) {
            draw_line(RAM, screen);
            RAM[LY] += 1;
            registers.cycles_counter -= 114;
        }
    } else {
        RAM[LY] = 0;
        registers.cycles_counter = 0;
    }

    if (RAM[LY] == 144 && !registers.VBLANKTriggered) {
        RAM[IF] |= VBLANK;
        registers.VBLANKTriggered = true;
    }

    if (RAM[LY] != 144) {
        registers.VBLANKTriggered = false;
    }

    if ((RAM[IF] & VBLANK) != 0) {
        if (registers.IME && (RAM[IE] & VBLANK) != 0) {
            registers.IME = false;
            RAM[IF] &= ~VBLANK;
            registers.cycles_counter += 2;
            CALL_Vector(registers.PC, RAM, registers.SP, VBLANK_VECTOR);
        }
    }

    if ((RAM[IF] & JOYPAD) != 0) {
        if (registers.IME && (RAM[IE] & JOYPAD) != 0) {
            registers.IME = false;
            RAM[IF] &= ~JOYPAD;
            registers.cycles_counter += 2;
            CALL_Vector(registers.PC, RAM, registers.SP, JOYPAD_VECTOR);
        }
    }

    if (RAM[LY] == 154) {
        RAM[LY] = 0;
        registers.cycles_counter++;
    }
}

void ECI(Registers& registers, std::vector<uint8_t>& RAM) {
    int complement = 0;

    registers.cycles_counter += instructions_cycles[RAM[registers.PC]];

    if (registers.preIME) {
        registers.IME = true;
        registers.preIME = false;
    }

    switch (RAM[registers.PC]) {
        case NOP_OP:
            NOP(registers.PC);
            break;
        case LD_BC_16_OP:
            LD_R16_16(registers.PC, registers.B, registers.C, RAM);
            break;
        case LD_ADBC_A_OP:
            LD_ADR16_R(registers.PC, RAM, registers.B, registers.C, registers.A);
            break;
        case INC_BC_OP:
            INC_R16(registers.PC, registers.B, registers.C);
            break;
        case INC_B_OP:
            INC_R8(registers.PC, registers.B, registers.F);
            break;
        case DEC_B_OP:
            DEC_R8(registers.PC, registers.B, registers.F);
            break;
        case LD_B_8_OP:
            LD_R8_8(registers.PC, registers.B, RAM);
            break;
        case RLCA_OP:
            RLCA(registers.PC, registers.A, registers.F);
            break;
        case LD_AD16_SP_OP:
            LD_AD16_SP(registers.PC, RAM, registers.SP);
            break;
        case ADD_HL_BC_OP:
            ADD_R16_R16(registers.PC, registers.H, registers.L, registers.B, registers.C, registers.F);
            break;
        case LD_A_ADBC_OP:
            LD_R8_ADR16(registers.PC, registers.A, RAM, registers.B, registers.C);
            break;
        case DEC_BC_OP:
            DEC_R16(registers.PC, registers.B, registers.C);
            break;
        case INC_C_OP:
            INC_R8(registers.PC, registers.C, registers.F);
            break;
        case DEC_C_OP:
            DEC_R8(registers.PC, registers.C, registers.F);
            break;
        case LD_C_8_OP:
            LD_R8_8(registers.PC, registers.C, RAM);
            break;
        case RRCA_OP:
            RRCA(registers.PC, registers.A, registers.F);
            break;
        case STOP_8_OP:
            STOP_8(registers.PC);
            break;
        case LD_DE_16_OP:
            LD_R16_16(registers.PC, registers.D, registers.E, RAM);
            break;
        case LD_ADDE_A_OP:
            LD_ADR16_R(registers.PC, RAM, registers.D, registers.E, registers.A);
            break;
        case INC_DE_OP:
            INC_R16(registers.PC, registers.D, registers.E);
            break;
        case INC_D_OP:
            INC_R8(registers.PC, registers.D, registers.F);
            break;
        case DEC_D_OP:
            DEC_R8(registers.PC, registers.D, registers.F);
            break;
        case LD_D_8_OP:
            LD_R8_8(registers.PC, registers.D, RAM);
            break;
        case RLA_OP:
            RLA(registers.PC, registers.A, registers.F);
            break;
        case JR_8_OP:
            JR_8(registers.PC, RAM);
            break;
        case ADD_HL_DE_OP:
            ADD_R16_R16(registers.PC, registers.H, registers.L, registers.D, registers.E, registers.F);
            break;
        case LD_A_ADDE_OP:
            LD_R8_ADR16(registers.PC, registers.A, RAM, registers.D, registers.E);
            break;
        case DEC_DE_OP:
            DEC_R16(registers.PC, registers.D, registers.E);
            break;
        case INC_E_OP:
            INC_R8(registers.PC, registers.E, registers.F);
            break;
        case DEC_E_OP:
            DEC_R8(registers.PC, registers.E, registers.F);
            break;
        case LD_E_8_OP:
            LD_R8_8(registers.PC, registers.E, RAM);
            break;
        case RRA_OP:
            RRA(registers.PC, registers.A, registers.F);
            break;
        case JR_NZ_8_OP:
            JR_NZ_8(registers.PC, registers.F, RAM);
            if ((registers.F & ZERO_FLAG) == 0) {
                complement = 1;
            }
            break;
        case LD_HL_16_OP:
            LD_R16_16(registers.PC, registers.H, registers.L, RAM);
            break;
        case LD_ADHLP_A_OP:
            LD_ADHL_I_A(registers.PC, RAM, registers.H, registers.L, registers.A);
            break;
        case INC_HL_OP:
            INC_R16(registers.PC, registers.H, registers.L);
            break;
        case INC_H_OP:
            INC_R8(registers.PC, registers.H, registers.F);
            break;
        case DEC_H_OP:
            DEC_R8(registers.PC, registers.H, registers.F);
            break;
        case LD_H_8_OP:
            LD_R8_8(registers.PC, registers.H, RAM);
            break;
        case DAA_OP:
            DAA(registers.PC, registers.A, registers.F);
            break;
        case JR_Z_8_OP:
            JR_Z_8(registers.PC, registers.F, RAM);
            if ((registers.F & ZERO_FLAG) != 0) {
                complement = 1;
            }
            break;
        case ADD_HL_HL_OP:
            ADD_R16_R16(registers.PC, registers.H, registers.L, registers.H, registers.L, registers.F);
            break;
        case LD_A_ADHLP_OP:
            LD_A_ADHL_I(registers.PC, RAM, registers.H, registers.L, registers.A);
            break;
        case DEC_HL_OP:
            DEC_R16(registers.PC, registers.H, registers.L);
            break;
        case INC_L_OP:
            INC_R8(registers.PC, registers.L, registers.F);
            break;
        case DEC_L_OP:
            DEC_R8(registers.PC, registers.L, registers.F);
            break;
        case LD_L_8_OP:
            LD_R8_8(registers.PC, registers.L, RAM);
            break;
        case CPL_OP:
            CPL(registers.PC, registers.A, registers.F);
            break;
        case JR_NC_8_OP:
            JR_NC_8(registers.PC, registers.F, RAM);
            if ((registers.F & CARRY_FLAG) == 0) {
                complement = 1;
            }
            break;
        case LD_SP_16_OP:
            LD_SP_16(registers.PC, registers.SP, RAM);
            break;
        case LD_ADHLM_A_OP:
            LD_ADHL_D_A(registers.PC, RAM, registers.H, registers.L, registers.A);
            break;
        case INC_SP_OP:
            INC_SP(registers.PC, registers.SP);
            break;
        case INC_ADHL_OP:
            INC_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F);
            break;
        case DEC_ADHL_OP:
            DEC_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F);
            break;
        case LD_ADHL_8_OP:
            LD_ADHL_8(registers.PC, RAM, registers.H, registers.L);
            break;
        case SCF_OP:
            SCF(registers.PC, registers.F);
            break;
        case JR_C_8_OP:
            JR_C_8(registers.PC, registers.F, RAM);
            if ((registers.F & CARRY_FLAG) != 0) {
                complement = 1;
            }
            break;
        case ADD_HL_SP_OP:
            ADD_HL_SP(registers.PC, registers.H, registers.L, registers.SP, registers.F);
            break;
        case LD_A_ADHLM_OP:
            LD_A_ADHL_D(registers.PC, RAM, registers.H, registers.L, registers.A);
            break;
        case DEC_SP_OP:
            DEC_SP(registers.PC, registers.SP);
            break;
        case INC_A_OP:
            INC_R8(registers.PC, registers.A, registers.F);
            break;
        case DEC_A_OP:
            DEC_R8(registers.PC, registers.A, registers.F);
            break;
        case LD_A_8_OP:
            LD_R8_8(registers.PC, registers.A, RAM);
            break;
        case CCF_OP:
            CCF(registers.PC, registers.F);
            break;
        case LD_B_B_OP:
            LD_R8_R8(registers.PC, registers.B, registers.B);
            break;
        case LD_B_C_OP:
            LD_R8_R8(registers.PC, registers.B, registers.C);
            break;
        case LD_B_D_OP:
            LD_R8_R8(registers.PC, registers.B, registers.D);
            break;
        case LD_B_E_OP:
            LD_R8_R8(registers.PC, registers.B, registers.E);
            break;
        case LD_B_H_OP:
            LD_R8_R8(registers.PC, registers.B, registers.H);
            break;
        case LD_B_L_OP:
            LD_R8_R8(registers.PC, registers.B, registers.L);
            break;
        case LD_B_ADHL_OP:
            LD_R8_ADR16(registers.PC, registers.B, RAM, registers.H, registers.L);
            break;
        case LD_B_A_OP:
            LD_R8_R8(registers.PC, registers.B, registers.A);
            break;
        case LD_C_B_OP:
            LD_R8_R8(registers.PC, registers.C, registers.B);
            break;
        case LD_C_C_OP:
            LD_R8_R8(registers.PC, registers.C, registers.C);
            break;
        case LD_C_D_OP:
            LD_R8_R8(registers.PC, registers.C, registers.D);
            break;
        case LD_C_E_OP:
            LD_R8_R8(registers.PC, registers.C, registers.E);
            break;
        case LD_C_H_OP:
            LD_R8_R8(registers.PC, registers.C, registers.H);
            break;
        case LD_C_L_OP:
            LD_R8_R8(registers.PC, registers.C, registers.L);
            break;
        case LD_C_ADHL_OP:
            LD_R8_ADR16(registers.PC, registers.C, RAM, registers.H, registers.L);
            break;
        case LD_C_A_OP:
            LD_R8_R8(registers.PC, registers.C, registers.A);
            break;
        case LD_D_B_OP:
            LD_R8_R8(registers.PC, registers.D, registers.B);
            break;
        case LD_D_C_OP:
            LD_R8_R8(registers.PC, registers.D, registers.C);
            break;
        case LD_D_D_OP:
            LD_R8_R8(registers.PC, registers.D, registers.D);
            break;
        case LD_D_E_OP:
            LD_R8_R8(registers.PC, registers.D, registers.E);
            break;
        case LD_D_H_OP:
            LD_R8_R8(registers.PC, registers.D, registers.H);
            break;
        case LD_D_L_OP:
            LD_R8_R8(registers.PC, registers.D, registers.L);
            break;
        case LD_D_ADHL_OP:
            LD_R8_ADR16(registers.PC, registers.D, RAM, registers.H, registers.L);
            break;
        case LD_D_A_OP:
            LD_R8_R8(registers.PC, registers.D, registers.A);
            break;
        case LD_E_B_OP:
            LD_R8_R8(registers.PC, registers.E, registers.B);
            break;
        case LD_E_C_OP:
            LD_R8_R8(registers.PC, registers.E, registers.C);
            break;
        case LD_E_D_OP:
            LD_R8_R8(registers.PC, registers.E, registers.D);
            break;
        case LD_E_E_OP:
            LD_R8_R8(registers.PC, registers.E, registers.E);
            break;
        case LD_E_H_OP:
            LD_R8_R8(registers.PC, registers.E, registers.H);
            break;
        case LD_E_L_OP:
            LD_R8_R8(registers.PC, registers.E, registers.L);
            break;
        case LD_E_ADHL_OP:
            LD_R8_ADR16(registers.PC, registers.E, RAM, registers.H, registers.L);
            break;
        case LD_E_A_OP:
            LD_R8_R8(registers.PC, registers.E, registers.A);
            break;
        case LD_H_B_OP:
            LD_R8_R8(registers.PC, registers.H, registers.B);
            break;
        case LD_H_C_OP:
            LD_R8_R8(registers.PC, registers.H, registers.C);
            break;
        case LD_H_D_OP:
            LD_R8_R8(registers.PC, registers.H, registers.D);
            break;
        case LD_H_E_OP:
            LD_R8_R8(registers.PC, registers.H, registers.E);
            break;
        case LD_H_H_OP:
            LD_R8_R8(registers.PC, registers.H, registers.H);
            break;
        case LD_H_L_OP:
            LD_R8_R8(registers.PC, registers.H, registers.L);
            break;
        case LD_H_ADHL_OP:
            LD_R8_ADR16(registers.PC, registers.H, RAM, registers.H, registers.L);
            break;
        case LD_H_A_OP:
            LD_R8_R8(registers.PC, registers.H, registers.A);
            break;
        case LD_L_B_OP:
            LD_R8_R8(registers.PC, registers.L, registers.B);
            break;
        case LD_L_C_OP:
            LD_R8_R8(registers.PC, registers.L, registers.C);
            break;
        case LD_L_D_OP:
            LD_R8_R8(registers.PC, registers.L, registers.D);
            break;
        case LD_L_E_OP:
            LD_R8_R8(registers.PC, registers.L, registers.E);
            break;
        case LD_L_H_OP:
            LD_R8_R8(registers.PC, registers.L, registers.H);
            break;
        case LD_L_L_OP:
            LD_R8_R8(registers.PC, registers.L, registers.L);
            break;
        case LD_L_ADHL_OP:
            LD_R8_ADR16(registers.PC, registers.L, RAM, registers.H, registers.L);
            break;
        case LD_L_A_OP:
            LD_R8_R8(registers.PC, registers.L, registers.A);
            break;
        case LD_ADHL_B_OP:
            LD_ADHL_R8(registers.PC, RAM, registers.H, registers.L, registers.B);
            break;
        case LD_ADHL_C_OP:
            LD_ADHL_R8(registers.PC, RAM, registers.H, registers.L, registers.C);
            break;
        case LD_ADHL_D_OP:
            LD_ADHL_R8(registers.PC, RAM, registers.H, registers.L, registers.D);
            break;
        case LD_ADHL_E_OP:
            LD_ADHL_R8(registers.PC, RAM, registers.H, registers.L, registers.E);
            break;
        case LD_ADHL_H_OP:
            LD_ADHL_R8(registers.PC, RAM, registers.H, registers.L, registers.H);
            break;
        case LD_ADHL_L_OP:
            LD_ADHL_R8(registers.PC, RAM, registers.H, registers.L, registers.L);
            break;
        case HALT_OP:
            HALT(registers.PC);
            break;
        case LD_ADHL_A_OP:
            LD_ADHL_R8(registers.PC, RAM, registers.H, registers.L, registers.A);
            break;
        case LD_A_B_OP:
            LD_R8_R8(registers.PC, registers.A, registers.B);
            break;
        case LD_A_C_OP:
            LD_R8_R8(registers.PC, registers.A, registers.C);
            break;
        case LD_A_D_OP:
            LD_R8_R8(registers.PC, registers.A, registers.D);
            break;
        case LD_A_E_OP:
            LD_R8_R8(registers.PC, registers.A, registers.E);
            break;
        case LD_A_H_OP:
            LD_R8_R8(registers.PC, registers.A, registers.H);
            break;
        case LD_A_L_OP:
            LD_R8_R8(registers.PC, registers.A, registers.L);
            break;
        case LD_A_ADHL_OP:
            LD_R8_ADR16(registers.PC, registers.A, RAM, registers.H, registers.L);
            break;
        case LD_A_A_OP:
            LD_R8_R8(registers.PC, registers.A, registers.A);
            break;
        case ADD_A_B_OP:
            ADD_A_8(registers.PC, registers.A, RAM, registers.F);
            break;
        case ADD_A_C_OP:
            ADD_R_R(registers.PC, registers.A, registers.C, registers.F);
            break;
        case ADD_A_D_OP:
            ADD_R_R(registers.PC, registers.A, registers.D, registers.F);
            break;
        case ADD_A_E_OP:
            ADD_R_R(registers.PC, registers.A, registers.E, registers.F);
            break;
        case ADD_A_H_OP:
            ADD_R_R(registers.PC, registers.A, registers.H, registers.F);
            break;
        case ADD_A_L_OP:
            ADD_R_R(registers.PC, registers.A, registers.L, registers.F);
            break;
        case ADD_A_ADHL_OP:
            ADD_R_ADHL(registers.PC, registers.A, RAM, registers.H, registers.L, registers.F);
            break;
        case ADD_A_A_OP:
            ADD_R_R(registers.PC, registers.A, registers.A, registers.F);
            break;
        case ADC_A_B_OP:
            ADC_R_R(registers.PC, registers.A, registers.B, registers.F);
            break;
        case ADC_A_C_OP:
            ADC_R_R(registers.PC, registers.A, registers.C, registers.F);
            break;
        case ADC_A_D_OP:
            ADC_R_R(registers.PC, registers.A, registers.D, registers.F);
            break;
        case ADC_A_E_OP:
            ADC_R_R(registers.PC, registers.A, registers.E, registers.F);
            break;
        case ADC_A_H_OP:
            ADC_R_R(registers.PC, registers.A, registers.H, registers.F);
            break;
        case ADC_A_L_OP:
            ADC_R_R(registers.PC, registers.A, registers.L, registers.F);
            break;
        case ADC_A_ADHL_OP:
            ADC_R_ADHL(registers.PC, registers.A, RAM, registers.H, registers.L, registers.F);
            break;
        case ADC_A_A_OP:
            ADC_R_R(registers.PC, registers.A, registers.A, registers.F);
            break;
        case SUB_A_B_OP:
            SUB_R_R(registers.PC, registers.A, registers.B, registers.F);
            break;
        case SUB_A_C_OP:
            SUB_R_R(registers.PC, registers.A, registers.C, registers.F);
            break;
        case SUB_A_D_OP:
            SUB_R_R(registers.PC, registers.A, registers.D, registers.F);
            break;
        case SUB_A_E_OP:
            SUB_R_R(registers.PC, registers.A, registers.E, registers.F);
            break;
        case SUB_A_H_OP:
            SUB_R_R(registers.PC, registers.A, registers.H, registers.F);
            break;
        case SUB_A_L_OP:
            SUB_R_R(registers.PC, registers.A, registers.L, registers.F);
            break;
        case SUB_A_ADHL_OP:
            SUB_R_ADHL(registers.PC, registers.A, RAM, registers.H, registers.L, registers.F);
            break;
        case SUB_A_A_OP:
            SUB_R_R(registers.PC, registers.A, registers.A, registers.F);
            break;
        case SBC_A_B_OP:
            SBC_R_R(registers.PC, registers.A, registers.B, registers.F);
            break;
        case SBC_A_C_OP:
            SBC_R_R(registers.PC, registers.A, registers.C, registers.F);
            break;
        case SBC_A_D_OP:
            SBC_R_R(registers.PC, registers.A, registers.D, registers.F);
            break;
        case SBC_A_E_OP:
            SBC_R_R(registers.PC, registers.A, registers.E, registers.F);
            break;
        case SBC_A_H_OP:
            SBC_R_R(registers.PC, registers.A, registers.H, registers.F);
            break;
        case SBC_A_L_OP:
            SBC_R_R(registers.PC, registers.A, registers.L, registers.F);
            break;
        case SBC_A_ADHL_OP:
            SBC_R_ADHL(registers.PC, registers.A, RAM, registers.H, registers.L, registers.F);
            break;
        case SBC_A_A_OP:
            SBC_R_R(registers.PC, registers.A, registers.A, registers.F);
            break;
        case AND_A_B_OP:
            AND_R_R(registers.PC, registers.A, registers.B, registers.F);
            break;
        case AND_A_C_OP:
            AND_R_R(registers.PC, registers.A, registers.C, registers.F);
            break;
        case AND_A_D_OP:
            AND_R_R(registers.PC, registers.A, registers.D, registers.F);
            break;
        case AND_A_E_OP:
            AND_R_R(registers.PC, registers.A, registers.E, registers.F);
            break;
        case AND_A_H_OP:
            AND_R_R(registers.PC, registers.A, registers.H, registers.F);
            break;
        case AND_A_L_OP:
            AND_R_R(registers.PC, registers.A, registers.L, registers.F);
            break;
        case AND_A_ADHL_OP:
            AND_R_ADHL(registers.PC, registers.A, RAM, registers.H, registers.L, registers.F);
            break;
        case AND_A_A_OP:
            AND_R_R(registers.PC, registers.A, registers.A, registers.F);
            break;
        case XOR_A_B_OP:
            XOR_R_R(registers.PC, registers.A, registers.B, registers.F);
            break;
        case XOR_A_C_OP:
            XOR_R_R(registers.PC, registers.A, registers.C, registers.F);
            break;
        case XOR_A_D_OP:
            XOR_R_R(registers.PC, registers.A, registers.D, registers.F);
            break;
        case XOR_A_E_OP:
            XOR_R_R(registers.PC, registers.A, registers.E, registers.F);
            break;
        case XOR_A_H_OP:
            XOR_R_R(registers.PC, registers.A, registers.H, registers.F);
            break;
        case XOR_A_L_OP:
            XOR_R_R(registers.PC, registers.A, registers.L, registers.F);
            break;
        case XOR_A_ADHL_OP:
            XOR_R_ADHL(registers.PC, registers.A, RAM, registers.H, registers.L, registers.F);
            break;
        case XOR_A_A_OP:
            XOR_R_R(registers.PC, registers.A, registers.A, registers.F);
            break;
        case OR_A_B_OP:
            OR_R_R(registers.PC, registers.A, registers.B, registers.F);
            break;
        case OR_A_C_OP:
            OR_R_R(registers.PC, registers.A, registers.C, registers.F);
            break;
        case OR_A_D_OP:
            OR_R_R(registers.PC, registers.A, registers.D, registers.F);
            break;
        case OR_A_E_OP:
            OR_R_R(registers.PC, registers.A, registers.E, registers.F);
            break;
        case OR_A_H_OP:
            OR_R_R(registers.PC, registers.A, registers.H, registers.F);
            break;
        case OR_A_L_OP:
            OR_R_R(registers.PC, registers.A, registers.L, registers.F);
            break;
        case OR_A_ADHL_OP:
            OR_R_ADHL(registers.PC, registers.A, RAM, registers.H, registers.L, registers.F);
            break;
        case OR_A_A_OP:
            OR_R_R(registers.PC, registers.A, registers.A, registers.F);
            break;
        case CP_A_B_OP:
            CP_R_R(registers.PC, registers.A, registers.B, registers.F);
            break;
        case CP_A_C_OP:
            CP_R_R(registers.PC, registers.A, registers.C, registers.F);
            break;
        case CP_A_D_OP:
            CP_R_R(registers.PC, registers.A, registers.D, registers.F);
            break;
        case CP_A_E_OP:
            CP_R_R(registers.PC, registers.A, registers.E, registers.F);
            break;
        case CP_A_H_OP:
            CP_R_R(registers.PC, registers.A, registers.H, registers.F);
            break;
        case CP_A_L_OP:
            CP_R_R(registers.PC, registers.A, registers.L, registers.F);
            break;
        case CP_A_ADHL_OP:
            CP_R_ADHL(registers.PC, registers.A, RAM, registers.H, registers.L, registers.F);
            break;
        case CP_A_A_OP:
            CP_R_R(registers.PC, registers.A, registers.A, registers.F);
            break;
        case RET_NZ_OP:
            RET_NZ(registers.PC, RAM, registers.SP, registers.F);
            if ((registers.F & ZERO_FLAG) == 0) {
                complement = 3;
            }
            break;
        case POP_BC_OP:
            POP_R16(registers.PC, RAM, registers.SP, registers.B, registers.C);
            break;
        case JP_NZ_16_OP:
            JP_NZ_16(registers.PC, RAM, registers.F);
            if ((registers.F & ZERO_FLAG) == 0) {
                complement = 1;
            }
            break;
        case JP_16_OP:
            JP_16(registers.PC, RAM);
            break;
        case CALL_NZ_16_OP:
            CALL_NZ_16(registers.PC, RAM, registers.SP, registers.F);
            if ((registers.F & ZERO_FLAG) == 0) {
                complement = 3;
            }
            break;
        case PUSH_BC_OP:
            PUSH_R16(registers.PC, RAM, registers.SP, registers.B, registers.C);
            break;
        case ADD_A_8_OP:
            ADD_A_8(registers.PC, registers.A, RAM, registers.F);
            break;
        case RST_00_OP:
            RST_AD(registers.PC, RAM, registers.SP, 0x00);
            break;
        case RET_Z_OP:
            RET_Z(registers.PC, RAM, registers.SP, registers.F);
            if ((registers.F & ZERO_FLAG) != 0) {
                complement = 3;
            }
            break;
        case RET_OP:
            RET(registers.PC, RAM, registers.SP);
            break;
        case JP_Z_16_OP:
            JP_Z_16(registers.PC, RAM, registers.F);
            if ((registers.F & ZERO_FLAG) != 0) {
                complement = 1;
            }
            break;
        case PREFIX_OP:
            registers.PC++;
            EPR(registers, RAM);
            break;
        case CALL_Z_16_OP:
            CALL_Z_16(registers.PC, RAM, registers.SP, registers.F);
            if ((registers.F & ZERO_FLAG) != 0) {
                complement = 3;
            }
            break;
        case CALL_16_OP:
            CALL_16(registers.PC, RAM, registers.SP);
            break;
        case ADC_A_8_OP:
            ADC_A_8(registers.PC, registers.A, RAM, registers.F);
            break;
        case RST_08_OP:
            RST_AD(registers.PC, RAM, registers.SP, 0x08);
            break;
        case RET_NC_OP:
            RET_NC(registers.PC, RAM, registers.SP, registers.F);
            if ((registers.F & CARRY_FLAG) == 0) {
                complement = 3;
            }
            break;
        case POP_DE_OP:
            POP_R16(registers.PC, RAM, registers.SP, registers.D, registers.E);
            break;
        case JP_NC_16_OP:
            JP_NC_16(registers.PC, RAM, registers.F);
            if ((registers.F & CARRY_FLAG) == 0) {
                complement = 1;
            }
            break;
        case CALL_NC_16_OP:
            CALL_NC_16(registers.PC, RAM, registers.SP, registers.F);
            if ((registers.F & CARRY_FLAG) == 0) {
                complement = 3;
            }
            break;
        case PUSH_DE_OP:
            PUSH_R16(registers.PC, RAM, registers.SP, registers.D, registers.E);
            break;
        case SUB_A_8_OP:
            SUB_A_8(registers.PC, registers.A, RAM, registers.F);
            break;
        case RST_10_OP:
            RST_AD(registers.PC, RAM, registers.SP, 0x10);
            break;
        case RET_C_OP:
            RET_C(registers.PC, RAM, registers.SP, registers.F);
            if ((registers.F & CARRY_FLAG) != 0) {
                complement = 3;
            }
            break;
        case RETI_OP:
            RETI(registers.PC, RAM, registers.SP, registers.IME);
            break;
        case JP_C_16_OP:
            JP_C_16(registers.PC, RAM, registers.F);
            if ((registers.F & CARRY_FLAG) != 0) {
                complement = 1;
            }
            break;
        case CALL_C_16_OP:
            CALL_C_16(registers.PC, RAM, registers.SP, registers.F);
            if ((registers.F & CARRY_FLAG) != 0) {
                complement = 3;
            }
            break;
        case SBC_A_8_OP:
            SBC_A_8(registers.PC, registers.A, RAM, registers.F);
            break;
        case RST_18_OP:
            RST_AD(registers.PC, RAM, registers.SP, 0x18);
            break;
        case LDH_AD8_A_OP:
            LDH_AD8_A(registers.PC, RAM, registers.A);
            break;
        case POP_HL_OP:
            POP_R16(registers.PC, RAM, registers.SP, registers.H, registers.L);
            break;
        case LDH_ADC_A_OP:
            LDH_ADC_A(registers.PC, RAM, registers.C, registers.A);
            break;
        case PUSH_HL_OP:
            PUSH_R16(registers.PC, RAM, registers.SP, registers.H, registers.L);
            break;
        case AND_A_8_OP:
            AND_A_8(registers.PC, registers.A, RAM, registers.F);
            break;
        case RST_20_OP:
            RST_AD(registers.PC, RAM, registers.SP, 0x20);
            break;
        case ADD_SP_8_OP:
            ADD_SP_8(registers.PC, registers.SP, RAM);
            break;
        case JP_HL_OP:
            JP_HL(registers.PC, registers.H, registers.L);
            break;
        case LD_AD16_A_OP:
            LD_AD16_A(registers.PC, RAM, registers.A);
            break;
        case XOR_A_8_OP:
            XOR_A_8(registers.PC, registers.A, RAM, registers.F);
            break;
        case RST_28_OP:
            RST_AD(registers.PC, RAM, registers.SP, 0x28);
            break;
        case LDH_A_AD8_OP:
            LDH_A_AD8(registers.PC, registers.A, RAM);
            break;
        case POP_AF_OP:
            POP_R16(registers.PC, RAM, registers.SP, registers.A, registers.F);
            break;
        case LDH_A_ADC_OP:
            LDH_A_ADC(registers.PC, registers.A, RAM, registers.C);
            break;
        case DI_OP:
            DI(registers.PC, registers.IME);
            break;
        case PUSH_AF_OP:
            PUSH_R16(registers.PC, RAM, registers.SP, registers.A, registers.F);
            break;
        case OR_A_8_OP:
            OR_A_8(registers.PC, registers.A, RAM, registers.F);
            break;
        case RST_30_OP:
            RST_AD(registers.PC, RAM, registers.SP, 0x30);
            break;
        case LD_HL_SP8_OP:
            LD_HL_SP8(registers.PC, registers.H, registers.L, registers.SP, RAM, registers.F);
            break;
        case LD_SP_HL_OP:
            LD_SP_HL(registers.PC, registers.SP, registers.H, registers.L);
            break;
        case LD_A_AD16_OP:
            LD_A_AD16(registers.PC, registers.A, RAM);
            break;
        case EI_OP:
            EI(registers.PC, registers.preIME);
            break;
        case CP_A_8_OP:
            CP_A_8(registers.PC, registers.A, RAM, registers.F);
            break;
        case RST_38_OP:
            RST_AD(registers.PC, RAM, registers.SP, 0x38);
            break;
        default:
            std::cout << "No correspondence found" << std::endl;
            registers.PC++;
            break;
    }

    registers.cycles_counter += complement;
}

void EPR(Registers& registers, std::vector<uint8_t>& RAM) {

    registers.cycles_counter += prefixed_cycles[RAM[registers.PC]];

    switch (RAM[registers.PC]) {
        case RLC_B_OP:
            RLC_R(registers.PC, registers.B, registers.F);
            break;
        case RLC_C_OP:
            RLC_R(registers.PC, registers.C, registers.F);
            break;
        case RLC_D_OP:
            RLC_R(registers.PC, registers.D, registers.F);
            break;
        case RLC_E_OP:
            RLC_R(registers.PC, registers.E, registers.F);
            break;
        case RLC_H_OP:
            RLC_R(registers.PC, registers.H, registers.F);
            break;
        case RLC_L_OP:
            RLC_R(registers.PC, registers.L, registers.F);
            break;
        case RLC_ADHL_OP:
            RLC_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F);
            break;
        case RLC_A_OP:
            RLC_R(registers.PC, registers.A, registers.F);
            break;
        case RRC_B_OP:
            RRC_R(registers.PC, registers.B, registers.F);
            break;
        case RRC_C_OP:
            RRC_R(registers.PC, registers.C, registers.F);
            break;
        case RRC_D_OP:
            RRC_R(registers.PC, registers.D, registers.F);
            break;
        case RRC_E_OP:
            RRC_R(registers.PC, registers.E, registers.F);
            break;
        case RRC_H_OP:
            RRC_R(registers.PC, registers.H, registers.F);
            break;
        case RRC_L_OP:
            RRC_R(registers.PC, registers.L, registers.F);
            break;
        case RRC_ADHL_OP:
            RRC_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F);
            break;
        case RRC_A_OP:
            RRC_R(registers.PC, registers.A, registers.F);
            break;
        case RL_B_OP:
            RL_R(registers.PC, registers.B, registers.F);
            break;
        case RL_C_OP:
            RL_R(registers.PC, registers.C, registers.F);
            break;
        case RL_D_OP:
            RL_R(registers.PC, registers.D, registers.F);
            break;
        case RL_E_OP:
            RL_R(registers.PC, registers.E, registers.F);
            break;
        case RL_H_OP:
            RL_R(registers.PC, registers.H, registers.F);
            break;
        case RL_L_OP:
            RL_R(registers.PC, registers.L, registers.F);
            break;
        case RL_ADHL_OP:
            RL_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F);
            break;
        case RL_A_OP:
            RL_R(registers.PC, registers.A, registers.F);
            break;
        case RR_B_OP:
            RR_R(registers.PC, registers.B, registers.F);
            break;
        case RR_C_OP:
            RR_R(registers.PC, registers.C, registers.F);
            break;
        case RR_D_OP:
            RR_R(registers.PC, registers.D, registers.F);
            break;
        case RR_E_OP:
            RR_R(registers.PC, registers.E, registers.F);
            break;
        case RR_H_OP:
            RR_R(registers.PC, registers.H, registers.F);
            break;
        case RR_L_OP:
            RR_R(registers.PC, registers.L, registers.F);
            break;
        case RR_ADHL_OP:
            RR_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F);
            break;
        case RR_A_OP:
            RR_R(registers.PC, registers.A, registers.F);
            break;
        case SLA_B_OP:
            SLA_R(registers.PC, registers.B, registers.F);
            break;
        case SLA_C_OP:
            SLA_R(registers.PC, registers.C, registers.F);
            break;
        case SLA_D_OP:
            SLA_R(registers.PC, registers.D, registers.F);
            break;
        case SLA_E_OP:
            SLA_R(registers.PC, registers.E, registers.F);
            break;
        case SLA_H_OP:
            SLA_R(registers.PC, registers.H, registers.F);
            break;
        case SLA_L_OP:
            SLA_R(registers.PC, registers.L, registers.F);
            break;
        case SLA_ADHL_OP:
            SLA_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F);
            break;
        case SLA_A_OP:
            SLA_R(registers.PC, registers.A, registers.F);
            break;
        case SRA_B_OP:
            SRA_R(registers.PC, registers.B, registers.F);
            break;
        case SRA_C_OP:
            SRA_R(registers.PC, registers.C, registers.F);
            break;
        case SRA_D_OP:
            SRA_R(registers.PC, registers.D, registers.F);
            break;
        case SRA_E_OP:
            SRA_R(registers.PC, registers.E, registers.F);
            break;
        case SRA_H_OP:
            SRA_R(registers.PC, registers.H, registers.F);
            break;
        case SRA_L_OP:
            SRA_R(registers.PC, registers.L, registers.F);
            break;
        case SRA_ADHL_OP:
            SRA_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F);
            break;
        case SRA_A_OP:
            SRA_R(registers.PC, registers.A, registers.F);
            break;
        case SWAP_B_OP:
            SWAP_R(registers.PC, registers.B, registers.F);
            break;
        case SWAP_C_OP:
            SWAP_R(registers.PC, registers.C, registers.F);
            break;
        case SWAP_D_OP:
            SWAP_R(registers.PC, registers.D, registers.F);
            break;
        case SWAP_E_OP:
            SWAP_R(registers.PC, registers.E, registers.F);
            break;
        case SWAP_H_OP:
            SWAP_R(registers.PC, registers.H, registers.F);
            break;
        case SWAP_L_OP:
            SWAP_R(registers.PC, registers.L, registers.F);
            break;
        case SWAP_ADHL_OP:
            SWAP_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F);
            break;
        case SWAP_A_OP:
            SWAP_R(registers.PC, registers.A, registers.F);
            break;
         case SRL_B_OP:
            SRL_R(registers.PC, registers.B, registers.F);
            break;
        case SRL_C_OP:
            SRL_R(registers.PC, registers.C, registers.F);
            break;
        case SRL_D_OP:
            SRL_R(registers.PC, registers.D, registers.F);
            break;
        case SRL_E_OP:
            SRL_R(registers.PC, registers.E, registers.F);
            break;
        case SRL_H_OP:
            SRL_R(registers.PC, registers.H, registers.F);
            break;
        case SRL_L_OP:
            SRL_R(registers.PC, registers.L, registers.F);
            break;
        case SRL_ADHL_OP:
            SRL_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F);
            break;
        case SRL_A_OP:
            SRL_R(registers.PC, registers.A, registers.F);
            break;
        case BIT_0_B_OP:
            BIT_N_R(registers.PC, registers.B, registers.F, 0);
            break;
        case BIT_0_C_OP:
            BIT_N_R(registers.PC, registers.C, registers.F, 0);
            break;
        case BIT_0_D_OP:
            BIT_N_R(registers.PC, registers.D, registers.F, 0);
            break;
        case BIT_0_E_OP:
            BIT_N_R(registers.PC, registers.E, registers.F, 0);
            break;
        case BIT_0_H_OP:
            BIT_N_R(registers.PC, registers.H, registers.F, 0);
            break;
        case BIT_0_L_OP:
            BIT_N_R(registers.PC, registers.L, registers.F, 0);
            break;
        case BIT_0_ADHL_OP:
            BIT_N_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F, 0);
            break;
        case BIT_0_A_OP:
            BIT_N_R(registers.PC, registers.A, registers.F, 0);
            break;
        case BIT_1_B_OP:
            BIT_N_R(registers.PC, registers.B, registers.F, 1);
            break;
        case BIT_1_C_OP:
            BIT_N_R(registers.PC, registers.C, registers.F, 1);
            break;
        case BIT_1_D_OP:
            BIT_N_R(registers.PC, registers.D, registers.F, 1);
            break;
        case BIT_1_E_OP:
            BIT_N_R(registers.PC, registers.E, registers.F, 1);
            break;
        case BIT_1_H_OP:
            BIT_N_R(registers.PC, registers.H, registers.F, 1);
            break;
        case BIT_1_L_OP:
            BIT_N_R(registers.PC, registers.L, registers.F, 1);
            break;
        case BIT_1_ADHL_OP:
            BIT_N_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F, 1);
            break;
        case BIT_1_A_OP:
            BIT_N_R(registers.PC, registers.A, registers.F, 1);
            break;
        case BIT_2_B_OP:
            BIT_N_R(registers.PC, registers.B, registers.F, 2);
            break;
        case BIT_2_C_OP:
            BIT_N_R(registers.PC, registers.C, registers.F, 2);
            break;
        case BIT_2_D_OP:
            BIT_N_R(registers.PC, registers.D, registers.F, 2);
            break;
        case BIT_2_E_OP:
            BIT_N_R(registers.PC, registers.E, registers.F, 2);
            break;
        case BIT_2_H_OP:
            BIT_N_R(registers.PC, registers.H, registers.F, 2);
            break;
        case BIT_2_L_OP:
            BIT_N_R(registers.PC, registers.L, registers.F, 2);
            break;
        case BIT_2_ADHL_OP:
            BIT_N_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F, 2);
            break;
        case BIT_2_A_OP:
            BIT_N_R(registers.PC, registers.A, registers.F, 2);
            break;
        case BIT_3_B_OP:
            BIT_N_R(registers.PC, registers.B, registers.F, 3);
            break;
        case BIT_3_C_OP:
            BIT_N_R(registers.PC, registers.C, registers.F, 3);
            break;
        case BIT_3_D_OP:
            BIT_N_R(registers.PC, registers.D, registers.F, 3);
            break;
        case BIT_3_E_OP:
            BIT_N_R(registers.PC, registers.E, registers.F, 3);
            break;
        case BIT_3_H_OP:
            BIT_N_R(registers.PC, registers.H, registers.F, 3);
            break;
        case BIT_3_L_OP:
            BIT_N_R(registers.PC, registers.L, registers.F, 3);
            break;
        case BIT_3_ADHL_OP:
            BIT_N_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F, 3);
            break;
        case BIT_3_A_OP:
            BIT_N_R(registers.PC, registers.A, registers.F, 3);
            break;
        case BIT_4_B_OP:
            BIT_N_R(registers.PC, registers.B, registers.F, 4);
            break;
        case BIT_4_C_OP:
            BIT_N_R(registers.PC, registers.C, registers.F, 4);
            break;
        case BIT_4_D_OP:
            BIT_N_R(registers.PC, registers.D, registers.F, 4);
            break;
        case BIT_4_E_OP:
            BIT_N_R(registers.PC, registers.E, registers.F, 4);
            break;
        case BIT_4_H_OP:
            BIT_N_R(registers.PC, registers.H, registers.F, 4);
            break;
        case BIT_4_L_OP:
            BIT_N_R(registers.PC, registers.L, registers.F, 4);
            break;
        case BIT_4_ADHL_OP:
            BIT_N_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F, 4);
            break;
        case BIT_4_A_OP:
            BIT_N_R(registers.PC, registers.A, registers.F, 4);
            break;
        case BIT_5_B_OP:
            BIT_N_R(registers.PC, registers.B, registers.F, 5);
            break;
        case BIT_5_C_OP:
            BIT_N_R(registers.PC, registers.C, registers.F, 5);
            break;
        case BIT_5_D_OP:
            BIT_N_R(registers.PC, registers.D, registers.F, 5);
            break;
        case BIT_5_E_OP:
            BIT_N_R(registers.PC, registers.E, registers.F, 5);
            break;
        case BIT_5_H_OP:
            BIT_N_R(registers.PC, registers.H, registers.F, 5);
            break;
        case BIT_5_L_OP:
            BIT_N_R(registers.PC, registers.L, registers.F, 5);
            break;
        case BIT_5_ADHL_OP:
            BIT_N_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F, 5);
            break;
        case BIT_5_A_OP:
            BIT_N_R(registers.PC, registers.A, registers.F, 5);
            break;
        case BIT_6_B_OP:
            BIT_N_R(registers.PC, registers.B, registers.F, 6);
            break;
        case BIT_6_C_OP:
            BIT_N_R(registers.PC, registers.C, registers.F, 6);
            break;
        case BIT_6_D_OP:
            BIT_N_R(registers.PC, registers.D, registers.F, 6);
            break;
        case BIT_6_E_OP:
            BIT_N_R(registers.PC, registers.E, registers.F, 6);
            break;
        case BIT_6_H_OP:
            BIT_N_R(registers.PC, registers.H, registers.F, 6);
            break;
        case BIT_6_L_OP:
            BIT_N_R(registers.PC, registers.L, registers.F, 6);
            break;
        case BIT_6_ADHL_OP:
            BIT_N_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F, 6);
            break;
        case BIT_6_A_OP:
            BIT_N_R(registers.PC, registers.A, registers.F, 6);
            break;
        case BIT_7_B_OP:
            BIT_N_R(registers.PC, registers.B, registers.F, 7);
            break;
        case BIT_7_C_OP:
            BIT_N_R(registers.PC, registers.C, registers.F, 7);
            break;
        case BIT_7_D_OP:
            BIT_N_R(registers.PC, registers.D, registers.F, 7);
            break;
        case BIT_7_E_OP:
            BIT_N_R(registers.PC, registers.E, registers.F, 7);
            break;
        case BIT_7_H_OP:
            BIT_N_R(registers.PC, registers.H, registers.F, 7);
            break;
        case BIT_7_L_OP:
            BIT_N_R(registers.PC, registers.L, registers.F, 7);
            break;
        case BIT_7_ADHL_OP:
            BIT_N_ADHL(registers.PC, RAM, registers.H, registers.L, registers.F, 7);
            break;
        case BIT_7_A_OP:
            BIT_N_R(registers.PC, registers.A, registers.F, 7);
            break;
        case RES_0_B_OP:
            RES_N_R(registers.PC, registers.B, 0);
            break;
        case RES_0_C_OP:
            RES_N_R(registers.PC, registers.C, 0);
            break;
        case RES_0_D_OP:
            RES_N_R(registers.PC, registers.D, 0);
            break;
        case RES_0_E_OP:
            RES_N_R(registers.PC, registers.E, 0);
            break;
        case RES_0_H_OP:
            RES_N_R(registers.PC, registers.H, 0);
            break;
        case RES_0_L_OP:
            RES_N_R(registers.PC, registers.L, 0);
            break;
        case RES_0_ADHL_OP:
            RES_N_ADHL(registers.PC, RAM, registers.H, registers.L, 0);
            break;
        case RES_0_A_OP:
            RES_N_R(registers.PC, registers.A, 0);
            break;
        case RES_1_B_OP:
            RES_N_R(registers.PC, registers.B, 1);
            break;
        case RES_1_C_OP:
            RES_N_R(registers.PC, registers.C, 1);
            break;
        case RES_1_D_OP:
            RES_N_R(registers.PC, registers.D, 1);
            break;
        case RES_1_E_OP:
            RES_N_R(registers.PC, registers.E, 1);
            break;
        case RES_1_H_OP:
            RES_N_R(registers.PC, registers.H, 1);
            break;
        case RES_1_L_OP:
            RES_N_R(registers.PC, registers.L, 1);
            break;
        case RES_1_ADHL_OP:
            RES_N_ADHL(registers.PC, RAM, registers.H, registers.L, 1);
            break;
        case RES_1_A_OP:
            RES_N_R(registers.PC, registers.A, 1);
            break;
        case RES_2_B_OP:
            RES_N_R(registers.PC, registers.B, 2);
            break;
        case RES_2_C_OP:
            RES_N_R(registers.PC, registers.C, 2);
            break;
        case RES_2_D_OP:
            RES_N_R(registers.PC, registers.D, 2);
            break;
        case RES_2_E_OP:
            RES_N_R(registers.PC, registers.E, 2);
            break;
        case RES_2_H_OP:
            RES_N_R(registers.PC, registers.H, 2);
            break;
        case RES_2_L_OP:
            RES_N_R(registers.PC, registers.L, 2);
            break;
        case RES_2_ADHL_OP:
            RES_N_ADHL(registers.PC, RAM, registers.H, registers.L, 2);
            break;
        case RES_2_A_OP:
            RES_N_R(registers.PC, registers.A, 2);
            break;
        case RES_3_B_OP:
            RES_N_R(registers.PC, registers.B, 3);
            break;
        case RES_3_C_OP:
            RES_N_R(registers.PC, registers.C, 3);
            break;
        case RES_3_D_OP:
            RES_N_R(registers.PC, registers.D, 3);
            break;
        case RES_3_E_OP:
            RES_N_R(registers.PC, registers.E, 3);
            break;
        case RES_3_H_OP:
            RES_N_R(registers.PC, registers.H, 3);
            break;
        case RES_3_L_OP:
            RES_N_R(registers.PC, registers.L, 3);
            break;
        case RES_3_ADHL_OP:
            RES_N_ADHL(registers.PC, RAM, registers.H, registers.L, 3);
            break;
        case RES_3_A_OP:
            RES_N_R(registers.PC, registers.A, 3);
            break;
        case RES_4_B_OP:
            RES_N_R(registers.PC, registers.B, 4);
            break;
        case RES_4_C_OP:
            RES_N_R(registers.PC, registers.C, 4);
            break;
        case RES_4_D_OP:
            RES_N_R(registers.PC, registers.D, 4);
            break;
        case RES_4_E_OP:
            RES_N_R(registers.PC, registers.E, 4);
            break;
        case RES_4_H_OP:
            RES_N_R(registers.PC, registers.H, 4);
            break;
        case RES_4_L_OP:
            RES_N_R(registers.PC, registers.L, 4);
            break;
        case RES_4_ADHL_OP:
            RES_N_ADHL(registers.PC, RAM, registers.H, registers.L, 4);
            break;
        case RES_4_A_OP:
            RES_N_R(registers.PC, registers.A, 4);
            break;
        case RES_5_B_OP:
            RES_N_R(registers.PC, registers.B, 5);
            break;
        case RES_5_C_OP:
            RES_N_R(registers.PC, registers.C, 5);
            break;
        case RES_5_D_OP:
            RES_N_R(registers.PC, registers.D, 5);
            break;
        case RES_5_E_OP:
            RES_N_R(registers.PC, registers.E, 5);
            break;
        case RES_5_H_OP:
            RES_N_R(registers.PC, registers.H, 5);
            break;
        case RES_5_L_OP:
            RES_N_R(registers.PC, registers.L, 5);
            break;
        case RES_5_ADHL_OP:
            RES_N_ADHL(registers.PC, RAM, registers.H, registers.L, 5);
            break;
        case RES_5_A_OP:
            RES_N_R(registers.PC, registers.A, 5);
            break;
        case RES_6_B_OP:
            RES_N_R(registers.PC, registers.B, 6);
            break;
        case RES_6_C_OP:
            RES_N_R(registers.PC, registers.C, 6);
            break;
        case RES_6_D_OP:
            RES_N_R(registers.PC, registers.D, 6);
            break;
        case RES_6_E_OP:
            RES_N_R(registers.PC, registers.E, 6);
            break;
        case RES_6_H_OP:
            RES_N_R(registers.PC, registers.H, 6);
            break;
        case RES_6_L_OP:
            RES_N_R(registers.PC, registers.L, 6);
            break;
        case RES_6_ADHL_OP:
            RES_N_ADHL(registers.PC, RAM, registers.H, registers.L, 6);
            break;
        case RES_6_A_OP:
            RES_N_R(registers.PC, registers.A, 6);
            break;
        case RES_7_B_OP:
            RES_N_R(registers.PC, registers.B, 7);
            break;
        case RES_7_C_OP:
            RES_N_R(registers.PC, registers.C, 7);
            break;
        case RES_7_D_OP:
            RES_N_R(registers.PC, registers.D, 7);
            break;
        case RES_7_E_OP:
            RES_N_R(registers.PC, registers.E, 7);
            break;
        case RES_7_H_OP:
            RES_N_R(registers.PC, registers.H, 7);
            break;
        case RES_7_L_OP:
            RES_N_R(registers.PC, registers.L, 7);
            break;
        case RES_7_ADHL_OP:
            RES_N_ADHL(registers.PC, RAM, registers.H, registers.L, 7);
            break;
        case RES_7_A_OP:
            RES_N_R(registers.PC, registers.A, 7);
            break;
        case SET_0_B_OP:
            SET_N_R(registers.PC, registers.B, 0);
            break;
        case SET_0_C_OP:
            SET_N_R(registers.PC, registers.C, 0);
            break;
        case SET_0_D_OP:
            SET_N_R(registers.PC, registers.D, 0);
            break;
        case SET_0_E_OP:
            SET_N_R(registers.PC, registers.E, 0);
            break;
        case SET_0_H_OP:
            SET_N_R(registers.PC, registers.H, 0);
            break;
        case SET_0_L_OP:
            SET_N_R(registers.PC, registers.L, 0);
            break;
        case SET_0_ADHL_OP:
            SET_N_ADHL(registers.PC, RAM, registers.H, registers.L, 0);
            break;
        case SET_0_A_OP:
            SET_N_R(registers.PC, registers.A, 0);
            break;
        case SET_1_B_OP:
            SET_N_R(registers.PC, registers.B, 1);
            break;
        case SET_1_C_OP:
            SET_N_R(registers.PC, registers.C, 1);
            break;
        case SET_1_D_OP:
            SET_N_R(registers.PC, registers.D, 1);
            break;
        case SET_1_E_OP:
            SET_N_R(registers.PC, registers.E, 1);
            break;
        case SET_1_H_OP:
            SET_N_R(registers.PC, registers.H, 1);
            break;
        case SET_1_L_OP:
            SET_N_R(registers.PC, registers.L, 1);
            break;
        case SET_1_ADHL_OP:
            SET_N_ADHL(registers.PC, RAM, registers.H, registers.L, 1);
            break;
        case SET_1_A_OP:
            SET_N_R(registers.PC, registers.A, 1);
            break;
        case SET_2_B_OP:
            SET_N_R(registers.PC, registers.B, 2);
            break;
        case SET_2_C_OP:
            SET_N_R(registers.PC, registers.C, 2);
            break;
        case SET_2_D_OP:
            SET_N_R(registers.PC, registers.D, 2);
            break;
        case SET_2_E_OP:
            SET_N_R(registers.PC, registers.E, 2);
            break;
        case SET_2_H_OP:
            SET_N_R(registers.PC, registers.H, 2);
            break;
        case SET_2_L_OP:
            SET_N_R(registers.PC, registers.L, 2);
            break;
        case SET_2_ADHL_OP:
            SET_N_ADHL(registers.PC, RAM, registers.H, registers.L, 2);
            break;
        case SET_2_A_OP:
            SET_N_R(registers.PC, registers.A, 2);
            break;
        case SET_3_B_OP:
            SET_N_R(registers.PC, registers.B, 3);
            break;
        case SET_3_C_OP:
            SET_N_R(registers.PC, registers.C, 3);
            break;
        case SET_3_D_OP:
            SET_N_R(registers.PC, registers.D, 3);
            break;
        case SET_3_E_OP:
            SET_N_R(registers.PC, registers.E, 3);
            break;
        case SET_3_H_OP:
            SET_N_R(registers.PC, registers.H, 3);
            break;
        case SET_3_L_OP:
            SET_N_R(registers.PC, registers.L, 3);
            break;
        case SET_3_ADHL_OP:
            SET_N_ADHL(registers.PC, RAM, registers.H, registers.L, 3);
            break;
        case SET_3_A_OP:
            SET_N_R(registers.PC, registers.A, 3);
            break;
        case SET_4_B_OP:
            SET_N_R(registers.PC, registers.B, 4);
            break;
        case SET_4_C_OP:
            SET_N_R(registers.PC, registers.C, 4);
            break;
        case SET_4_D_OP:
            SET_N_R(registers.PC, registers.D, 4);
            break;
        case SET_4_E_OP:
            SET_N_R(registers.PC, registers.E, 4);
            break;
        case SET_4_H_OP:
            SET_N_R(registers.PC, registers.H, 4);
            break;
        case SET_4_L_OP:
            SET_N_R(registers.PC, registers.L, 4);
            break;
        case SET_4_ADHL_OP:
            SET_N_ADHL(registers.PC, RAM, registers.H, registers.L, 4);
            break;
        case SET_4_A_OP:
            SET_N_R(registers.PC, registers.A, 4);
            break;
        case SET_5_B_OP:
            SET_N_R(registers.PC, registers.B, 5);
            break;
        case SET_5_C_OP:
            SET_N_R(registers.PC, registers.C, 5);
            break;
        case SET_5_D_OP:
            SET_N_R(registers.PC, registers.D, 5);
            break;
        case SET_5_E_OP:
            SET_N_R(registers.PC, registers.E, 5);
            break;
        case SET_5_H_OP:
            SET_N_R(registers.PC, registers.H, 5);
            break;
        case SET_5_L_OP:
            SET_N_R(registers.PC, registers.L, 5);
            break;
        case SET_5_ADHL_OP:
            SET_N_ADHL(registers.PC, RAM, registers.H, registers.L, 5);
            break;
        case SET_5_A_OP:
            SET_N_R(registers.PC, registers.A, 5);
            break;
        case SET_6_B_OP:
            SET_N_R(registers.PC, registers.B, 6);
            break;
        case SET_6_C_OP:
            SET_N_R(registers.PC, registers.C, 6);
            break;
        case SET_6_D_OP:
            SET_N_R(registers.PC, registers.D, 6);
            break;
        case SET_6_E_OP:
            SET_N_R(registers.PC, registers.E, 6);
            break;
        case SET_6_H_OP:
            SET_N_R(registers.PC, registers.H, 6);
            break;
        case SET_6_L_OP:
            SET_N_R(registers.PC, registers.L, 6);
            break;
        case SET_6_ADHL_OP:
            SET_N_ADHL(registers.PC, RAM, registers.H, registers.L, 6);
            break;
        case SET_6_A_OP:
            SET_N_R(registers.PC, registers.A, 6);
            break;
        case SET_7_B_OP:
            SET_N_R(registers.PC, registers.B, 7);
            break;
        case SET_7_C_OP:
            SET_N_R(registers.PC, registers.C, 7);
            break;
        case SET_7_D_OP:
            SET_N_R(registers.PC, registers.D, 7);
            break;
        case SET_7_E_OP:
            SET_N_R(registers.PC, registers.E, 7);
            break;
        case SET_7_H_OP:
            SET_N_R(registers.PC, registers.H, 7);
            break;
        case SET_7_L_OP:
            SET_N_R(registers.PC, registers.L, 7);
            break;
        case SET_7_ADHL_OP:
            SET_N_ADHL(registers.PC, RAM, registers.H, registers.L, 7);
            break;
        case SET_7_A_OP:
            SET_N_R(registers.PC, registers.A, 7);
            break;
    }

}