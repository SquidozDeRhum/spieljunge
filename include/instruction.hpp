#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdint>

void NOP(uint16_t& PC);
void STOP_8(uint16_t& PC);
void HALT(uint16_t& PC);

void LD_R8_8(uint16_t& PC, uint8_t& R, std::vector<uint8_t>& RAM);
void LD_R8_ADR16(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t R21, uint8_t R22);
void LD_R8_R8(uint16_t& PC, uint8_t& R1, uint8_t R2);
void LD_R16_16(uint16_t& PC, uint8_t& R1, uint8_t& R2, std::vector<uint8_t>& RAM);
void LD_ADR16_R(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t R11, uint8_t R12, uint8_t R2);
void LD_AD16_SP(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t SP);
void LD_SP_16(uint16_t& PC, uint16_t& SP, std::vector<uint8_t>& RAM);
void LD_AD16_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t A);
void LD_ADHL_I_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A);
void LD_ADHL_D_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A);
void LD_A_ADHL_I(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A);
void LD_A_ADHL_D(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t A);
void LD_ADHL_R8(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t R);
void LD_ADHL_8(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L);
void LD_HL_SP8(uint16_t& PC, uint8_t& H, uint8_t& L, uint16_t SP, std::vector<uint8_t>& RAM, uint8_t& F);
void LD_SP_HL(uint16_t& PC, uint16_t& SP, uint8_t H, uint8_t L);
void LD_A_AD16(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM);

void LDH_AD8_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t A);
void LDH_ADC_A(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t C, uint8_t A);
void LDH_A_AD8(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM);
void LDH_A_ADC(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM, uint8_t C);

void INC_R8(uint16_t& PC, uint8_t& R, uint8_t& F);
void INC_R16(uint16_t& PC, uint8_t& R1, uint8_t& R2);
void INC_ADHL(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);
void INC_SP(uint16_t& PC, uint16_t& SP);

void DEC_R8(uint16_t& PC, uint8_t& R, uint8_t& F);
void DEC_R16(uint16_t& PC, uint8_t& R1, uint8_t& R2);
void DEC_ADHL(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);
void DEC_SP(uint16_t& PC, uint16_t& SP);

void ADD_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void ADD_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);
void ADD_R16_R16(uint16_t& PC, uint8_t& R11, uint8_t& R12, uint8_t R21, uint8_t R22, uint8_t& F);
void ADD_A_8(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM, uint8_t& F);
void ADD_SP_8(uint16_t& PC, uint16_t& SP, std::vector<uint8_t>& RAM);
void ADD_HL_SP(uint16_t& PC, uint8_t& H, uint8_t& L, uint16_t SP, uint8_t& F);

// stands for ADD with carry
void ADC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void ADC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);
void ADC_A_8(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM, uint8_t& F);

void SUB_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void SUB_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);
void SUB_A_8(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM, uint8_t& F);

// stands for SUB with carry
void SBC_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void SBC_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);
void SBC_A_8(uint16_t& PC, uint8_t&A, std::vector<uint8_t>& RAM, uint8_t& F);

void AND_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void AND_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F);
void AND_A_8(uint16_t& PC, uint8_t& A, std::vector<uint8_t>& RAM, uint8_t& F);

void XOR_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void XOR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F);
void XOR_A_8(uint16_t& PC, uint8_t A, std::vector<uint8_t>& RAM, uint8_t& F);

void OR_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void OR_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F);
void OR_A_8(uint16_t& PC, uint8_t A, std::vector<uint8_t>& RAM, uint8_t& F);

void CP_R_R(uint16_t& PC, uint8_t& R1, uint8_t& R2, uint8_t& F);
void CP_R_ADHL(uint16_t& PC, uint8_t& R1, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L,uint8_t& F);
void CP_A_8(uint16_t& PC, uint8_t A, std::vector<uint8_t>& RAM, uint8_t& F);

void RLCA(uint16_t& PC, uint8_t& A, uint8_t& F);
void RRCA(uint16_t& PC, uint8_t& A, uint8_t& F);
void RLA(uint16_t& PC, uint8_t& A, uint8_t& F);
void RRA(uint16_t& PC, uint8_t& A, uint8_t& F);

void CPL(uint16_t& PC, uint8_t& R, uint8_t& F);

void SCF(uint16_t& PC, uint8_t& F); // Stands for Set Carry Flag
void CCF(uint16_t& PC, uint8_t& F); // Stands for Complement Carry Flag, it flips the carry flag

void JR_8(uint16_t& PC, std::vector<uint8_t>& RAM);
void JR_Z_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM);
void JR_NZ_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM);
void JR_C_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM);
void JR_NC_8(uint16_t& PC, uint8_t& F, std::vector<uint8_t>& RAM);

void JP_16(uint16_t& PC, std::vector<uint8_t>& RAM);
void JP_Z_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t F);
void JP_NZ_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t F);
void JP_C_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t F);
void JP_NC_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint8_t F);
void JP_HL(uint16_t& PC, uint8_t H, uint8_t L);

void DAA(uint16_t& PC, uint8_t& A, uint8_t& F);

void RST_AD(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t AD);

void CALL_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP);
void CALL_Z_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F);
void CALL_NZ_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F);
void CALL_C_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F);
void CALL_NC_16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F);

void RET(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP);
void RET_Z(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F);
void RET_NZ(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F);
void RET_C(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F);
void RET_NC(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t F);

void DI(uint16_t& PC);
void EI(uint16_t& PC);
void RETI(uint16_t& PC);

void POP_R16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t& R1, uint8_t& R2);
void PUSH_R16(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint8_t& R1, uint8_t& R2);

void PREFIX(uint16_t& PC);