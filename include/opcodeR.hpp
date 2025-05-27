#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdint>

void NOP(uint16_t& PC);
void STOP_8(uint16_t& PC);
void HALT(uint16_t& PC);

void LD_R_8(uint16_t& PC, uint8_t& R, std::vector<uint8_t>& RAM);
void LD_R_ADR16(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t R21, uint8_t R22);
void LD_R16_16(uint16_t& PC, uint8_t& R1, uint8_t& R2, std::vector<uint8_t>& RAM);
void LD_ADR16_R(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t R11, uint8_t R12, uint8_t R2);
void LD_AD16_SP(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t SP);
void LD_AD16_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t A);
void LD_ADHL_I_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A);
void LD_ADHL_D_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A);
void LD_A_ADHL_I(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A);
void LD_A_ADHL_D(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A);
void LD_ADHL_8(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L);

void INC_R8(uint16_t& PC, uint8_t& R, uint8_t& F);
void INC_R16(uint16_t& PC, uint8_t& R1, uint8_t& R2);
void INC_ADHL(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void DEC_R8(uint16_t& PC, uint8_t& R, uint8_t& F);
void DEC_R16(uint16_t& PC, uint8_t& R1, uint8_t& R2);
void DEC_ADHL(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void ADD_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void ADD_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);
void ADD_R16_R16(uint16_t& PC, uint8_t& R11, uint8_t& R12, uint8_t R21, uint8_t R22, uint8_t& F);

// stands for ADD with carry
void ADC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void ADC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void SUB_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void SUB_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

// stands for SUB with carry
void SBC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void SBC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void AND_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void AND_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);

void XOR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);
void XOR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void OR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);
void OR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void CP_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);
void CP_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void RLC_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void RRC_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void RL_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void RR_R(uint16_t& PC, uint8_t& R, uint8_t& F);

void CPL(uint16_t& PC, uint8_t& A, uint8_t& F);
void SCF(uint16_t& PC, uint8_t& F); // Stands for Set Carry Flag
void CCF(uint16_t& PC, uint8_t& F); // Stands for Complement Carry Flag

void JR_8(uint16_t& PC, std::vector<uint8_t>& RAM);
void JR_NZ_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM);
void JR_Z_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM);
void JR_NC_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM);
void JR_C_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM);

void DAA(uint16_t& PC, uint8_t& A, uint8_t& F);

// Goofy instructions

void RET_NZ(uint16_t& PC);
void POP_BC(uint16_t& PC);
void JP_NZ_16(uint16_t& PC);
void JP_16(uint16_t& PC);
void CALL_NZ_16(uint16_t& PC);
void PUSH_BC(uint16_t& PC);
void ADD_A_8(uint16_t& PC);
void RST_00(uint16_t& PC);
void RET_Z(uint16_t& PC);
void RET(uint16_t& PC);
void JP_Z_16(uint16_t& PC);
void PREFIX(uint16_t& PC);
void CALL_Z_16(uint16_t& PC);
void CALL_16(uint16_t& PC);
void ADC_A_8(uint16_t& PC);
void RST_08(uint16_t& PC);
void RET_NC(uint16_t& PC);
void POP_DE(uint16_t& PC);
void JP_NC_16(uint16_t& PC);
void CALL_NC_16(uint16_t& PC);
void PUSH_DE(uint16_t& PC);
void SUB_A_8(uint16_t& PC);
void RST_10(uint16_t& PC);
void RET_C(uint16_t& PC);
void RETI(uint16_t& PC);
void JP_C_16(uint16_t& PC);
void CALL_C_16(uint16_t& PC);
void SBC_A_8(uint16_t& PC);
void RST_18(uint16_t& PC);
void LDH_AD8_A(uint16_t& PC);
void POP_HL(uint16_t& PC);
void LDH_ADC_A(uint16_t& PC);
void PUSH_HL(uint16_t& PC);
void AND_A_8(uint16_t& PC);
void RST_20(uint16_t& PC);
void ADD_SP_8(uint16_t& PC);
void JP_HL(uint16_t& PC);
void LD_AD16_A(uint16_t& PC);
void XOR_A_8(uint16_t& PC);
void RST_28(uint16_t& PC);
void LDH_A_8(uint16_t& PC);
void POP_AF(uint16_t& PC);
void LDH_A_AD_C(uint16_t& PC);
void DI(uint16_t& PC);
void PUSH_AF(uint16_t& PC);
void OR_A_8(uint16_t& PC);
void RST_30(uint16_t& PC);
void LD_HL_SP8(uint16_t& PC);
void LD_SP_HL(uint16_t& PC);
void LD_A_16(uint16_t& PC);
void EI(uint16_t& PC);
void CP_A_8(uint16_t& PC);
void RST_38(uint16_t& PC);