#pragma once

/*
* Here are shown all principals GB OpCode
* Prefexided OpCode are in prefixed.hpp
* Maybe similar OpCode will be regrouped in the future
*/

// 0x00 - 0x0F
void NOP(u_int16_t& PC);
void LD_BC_16(u_int16_t& PC, u_int8_t& B, u_int8_t& C, std::vector<u_int8_t>& RAM);
void LD_ADBC_A(u_int16_t& PC, std::vector<u_int8_t>& RAM, u_int8_t B, u_int8_t C, u_int8_t A);
void INC_BC(u_int16_t& PC, u_int8_t& B, u_int8_t& C);
void INC_B(u_int16_t& PC, u_int8_t& B, u_int8_t& F);
void DEC_B(u_int16_t& PC, u_int8_t& B, u_int8_t& F);
void LD_B_8(u_int16_t& PC, u_int8_t& B, std::vector<u_int8_t>& RAM);
void RLCA(u_int16_t& PC, u_int8_t& A, u_int8_t& F);
void LD_AD16_SP(u_int16_t& PC, std::vector<u_int8_t>& RAM, u_int16_t SP);
void ADD_HL_BC(u_int16_t& PC, u_int8_t& H, u_int8_t& L, u_int8_t B, u_int8_t C, u_int8_t& F);
void LD_A_ADBC(u_int16_t& PC, u_int8_t& A, std::vector<u_int8_t>& RAM, u_int8_t B, u_int8_t C);
void DEC_BC(u_int16_t& PC, u_int8_t& B, u_int8_t& C);
void INC_C(u_int16_t& PC, u_int8_t& C, u_int8_t& F);
void DEC_C(u_int16_t& PC, u_int8_t& B, u_int8_t& F);
void LD_C_8(u_int16_t& PC, u_int8_t& C, std::vector<u_int8_t>& RAM);
void RRCA(u_int16_t& PC, u_int8_t& A, u_int8_t& F);

// 0x10 - 0x1F
void STOP_8(u_int16_t& PC);
void LD_DE_16(u_int16_t& PC, u_int8_t& D, u_int8_t& E, std::vector<u_int8_t>& RAM);
void LD_ADDE_A(u_int16_t& PC, std::vector<u_int8_t>& RAM, u_int8_t D, u_int8_t E, u_int8_t A);
void INC_DE(u_int16_t& PC, u_int8_t& D, u_int8_t& E);
void INC_D(u_int16_t& PC, u_int8_t& D, u_int8_t& F);
void DEC_D(u_int16_t& PC, u_int8_t& D, u_int8_t& F);
void LD_D_8(u_int16_t& PC, u_int8_t& D, std::vector<u_int8_t>& RAM);
void RLA(u_int16_t& PC, u_int8_t& A, u_int8_t& F);
void JR_8(u_int16_t& PC, std::vector<u_int8_t>& RAM); // Stands for Jump Relative
void ADD_HL_DE(u_int16_t& PC, u_int8_t& H, u_int8_t& L, u_int8_t D, u_int8_t E, u_int8_t& F);
void LD_A_ADDE(u_int16_t& PC, u_int8_t& A, std::vector<u_int8_t>& RAM, u_int8_t D, u_int8_t E);
void DEC_DE(u_int16_t& PC, u_int8_t& D, u_int8_t& E);
void INC_E(u_int16_t& PC, u_int8_t& E, u_int8_t& F);
void DEC_E(u_int16_t& PC, u_int8_t& E, u_int8_t& F);
void LD_E_8(u_int16_t& PC, u_int8_t& E, std::vector<u_int8_t>& RAM);
void RRA(u_int16_t& PC, u_int8_t& A, u_int8_t& F);

// 0x20 - 0x2F
void JR_NZ_8(u_int16_t& PC, u_int8_t& F, std::vector<u_int8_t>& RAM);
void LD_HL_16(u_int16_t& PC, u_int8_t& H, u_int8_t& L, std::vector<u_int8_t>& RAM);
void LD_ADHL_I_A(u_int16_t& PC, std::vector<u_int8_t>& RAM, u_int8_t& H, u_int8_t& L, u_int8_t A);
void INC_HL(u_int16_t& PC, u_int8_t& H, u_int8_t& L);
void INC_H(u_int16_t& PC, u_int8_t& H, u_int8_t& F);
void DEC_H(u_int16_t& PC);
void LD_H_8(u_int16_t& PC);
void DAA(u_int16_t& PC);
void JR_Z_8(u_int16_t& PC); 
void ADD_HL_HL(u_int16_t& PC);
void LD_A_ADHL_I(u_int16_t& PC);
void DEC_HL(u_int16_t& PC);
void INC_L(u_int16_t& PC);
void DEC_L(u_int16_t& PC);
void LD_L_8(u_int16_t& PC);
void CPL(u_int16_t& PC);

// 0x30 - 0x3F
void JR_NC_8(u_int16_t& PC);
void LD_SP_16(u_int16_t& PC);
void LD_ADHL_D_A(u_int16_t& PC);
void INC_SP(u_int16_t& PC);
void INC_ADHL(u_int16_t& PC);
void DEC_ADHL(u_int16_t& PC);
void LD_ADHL_8(u_int16_t& PC);
void SCF(u_int16_t& PC);
void JR_C_8(u_int16_t& PC);
void ADD_HL_SP(u_int16_t& PC);
void LD_A_ADHL_D(u_int16_t& PC);
void DEC_SP(u_int16_t& PC);
void INC_A(u_int16_t& PC);
void DEC_A(u_int16_t& PC);
void LD_A_8(u_int16_t& PC);
void CCF(u_int16_t& PC);

// 0x40 - 0x4F
void LD_B_B(u_int16_t& PC);
void LD_B_C(u_int16_t& PC);
void LD_B_D(u_int16_t& PC);
void LD_B_E(u_int16_t& PC);
void LD_B_H(u_int16_t& PC);
void LD_B_L(u_int16_t& PC);
void LD_B_ADHL(u_int16_t& PC);
void LD_B_A(u_int16_t& PC);
void LD_C_B(u_int16_t& PC);
void LD_C_C(u_int16_t& PC);
void LD_C_D(u_int16_t& PC);
void LD_C_E(u_int16_t& PC);
void LD_C_H(u_int16_t& PC);
void LD_C_L(u_int16_t& PC);
void LD_C_ADHL(u_int16_t& PC);
void LD_C_A(u_int16_t& PC);

// 0x50 - 0x5F
void LD_D_B(u_int16_t& PC);
void LD_D_C(u_int16_t& PC);
void LD_D_D(u_int16_t& PC);
void LD_D_E(u_int16_t& PC);
void LD_D_H(u_int16_t& PC);
void LD_D_L(u_int16_t& PC);
void LD_D_ADHL(u_int16_t& PC);
void LD_D_A(u_int16_t& PC);
void LD_E_B(u_int16_t& PC);
void LD_E_C(u_int16_t& PC);
void LD_E_D(u_int16_t& PC);
void LD_E_E(u_int16_t& PC);
void LD_E_H(u_int16_t& PC);
void LD_E_L(u_int16_t& PC);
void LD_E_ADHL(u_int16_t& PC);
void LD_E_A(u_int16_t& PC);

// 0x60 - 0x6F
void LD_H_B(u_int16_t& PC);
void LD_H_C(u_int16_t& PC);
void LD_H_D(u_int16_t& PC);
void LD_H_E(u_int16_t& PC);
void LD_H_H(u_int16_t& PC);
void LD_H_L(u_int16_t& PC);
void LD_H_ADHL(u_int16_t& PC);
void LD_H_A(u_int16_t& PC);
void LD_L_B(u_int16_t& PC);
void LD_L_C(u_int16_t& PC);
void LD_L_D(u_int16_t& PC);
void LD_L_E(u_int16_t& PC);
void LD_L_H(u_int16_t& PC);
void LD_L_L(u_int16_t& PC);
void LD_L_ADHL(u_int16_t& PC);
void LD_L_A(u_int16_t& PC);

// 0x70 - 0x7F
void LD_ADHL_B(u_int16_t& PC);
void LD_ADHL_C(u_int16_t& PC);
void LD_ADHL_D(u_int16_t& PC);
void LD_ADHL_E(u_int16_t& PC);
void LD_ADHL_H(u_int16_t& PC);
void LD_ADHL_L(u_int16_t& PC);
void HALT(u_int16_t& PC);
void LD_ADHL_A(u_int16_t& PC);
void LD_A_B(u_int16_t& PC);
void LD_A_C(u_int16_t& PC);
void LD_A_D(u_int16_t& PC);
void LD_A_E(u_int16_t& PC);
void LD_A_H(u_int16_t& PC);
void LD_A_L(u_int16_t& PC);
void LD_A_ADHL(u_int16_t& PC);
void LD_A_A(u_int16_t& PC);

// 0x80 - 0x8F
void ADD_A_B(u_int16_t& PC);
void ADD_A_C(u_int16_t& PC);
void ADD_A_D(u_int16_t& PC);
void ADD_A_E(u_int16_t& PC);
void ADD_A_H(u_int16_t& PC);
void ADD_A_L(u_int16_t& PC);
void ADD_A_ADHL(u_int16_t& PC);
void ADD_A_A(u_int16_t& PC);
void ADC_A_B(u_int16_t& PC);
void ADC_A_C(u_int16_t& PC);
void ADC_A_D(u_int16_t& PC);
void ADC_A_E(u_int16_t& PC);
void ADC_A_H(u_int16_t& PC);
void ADC_A_L(u_int16_t& PC);
void ADC_A_ADHL(u_int16_t& PC);
void ADC_A_A(u_int16_t& PC);

// 0x90 - 0x9F
void SUB_A_B(u_int16_t& PC);
void SUB_A_C(u_int16_t& PC);
void SUB_A_D(u_int16_t& PC);
void SUB_A_E(u_int16_t& PC);
void SUB_A_H(u_int16_t& PC);
void SUB_A_L(u_int16_t& PC);
void SUB_A_ADHL(u_int16_t& PC);
void SUB_A_A(u_int16_t& PC);
void SBC_A_B(u_int16_t& PC);
void SBC_A_C(u_int16_t& PC);
void SBC_A_D(u_int16_t& PC);
void SBC_A_E(u_int16_t& PC);
void SBC_A_H(u_int16_t& PC);
void SBC_A_L(u_int16_t& PC);
void SBC_A_ADHL(u_int16_t& PC);
void SBC_A_A(u_int16_t& PC);

// 0xA0 - 0xAF
void AND_A_B(u_int16_t& PC);
void AND_A_C(u_int16_t& PC);
void AND_A_D(u_int16_t& PC);
void AND_A_E(u_int16_t& PC);
void AND_A_H(u_int16_t& PC);
void AND_A_L(u_int16_t& PC);
void AND_A_ADHL(u_int16_t& PC);
void AND_A_A(u_int16_t& PC);
void XOR_A_B(u_int16_t& PC);
void XOR_A_C(u_int16_t& PC);
void XOR_A_D(u_int16_t& PC);
void XOR_A_E(u_int16_t& PC);
void XOR_A_H(u_int16_t& PC);
void XOR_A_L(u_int16_t& PC);
void XOR_A_ADHL(u_int16_t& PC);
void XOR_A_A(u_int16_t& PC);

// 0xB0 - 0xBF
void OR_A_B(u_int16_t& PC);
void OR_A_C(u_int16_t& PC);
void OR_A_D(u_int16_t& PC);
void OR_A_E(u_int16_t& PC);
void OR_A_H(u_int16_t& PC);
void OR_A_L(u_int16_t& PC);
void OR_A_ADHL(u_int16_t& PC);
void OR_A_A(u_int16_t& PC);
void CP_A_B(u_int16_t& PC);
void CP_A_C(u_int16_t& PC);
void CP_A_D(u_int16_t& PC);
void CP_A_E(u_int16_t& PC);
void CP_A_H(u_int16_t& PC);
void CP_A_L(u_int16_t& PC);
void CP_A_ADHL(u_int16_t& PC);
void CP_A_A(u_int16_t& PC);

// 0xC0 - 0xCF
void RET_NZ(u_int16_t& PC);
void POP_BC(u_int16_t& PC);
void JP_NZ_16(u_int16_t& PC);
void JP_16(u_int16_t& PC);
void CALL_NZ_16(u_int16_t& PC);
void PUSH_BC(u_int16_t& PC);
void ADD_A_8(u_int16_t& PC);
void RST_00(u_int16_t& PC);
void RET_Z(u_int16_t& PC);
void RET(u_int16_t& PC);
void JP_Z_16(u_int16_t& PC);
void PREFIX(u_int16_t& PC);
void CALL_Z_16(u_int16_t& PC);
void CALL_16(u_int16_t& PC);
void ADC_A_8(u_int16_t& PC);
void RST_08(u_int16_t& PC);

// 0xD0 - 0xDF
void RET_NC(u_int16_t& PC);
void POP_DE(u_int16_t& PC);
void JP_NC_16(u_int16_t& PC);
// No correspondence
void CALL_NC_16(u_int16_t& PC);
void PUSH_DE(u_int16_t& PC);
void SUB_A_8(u_int16_t& PC);
void RST_10(u_int16_t& PC);
void RET_C(u_int16_t& PC);
void RETI(u_int16_t& PC);
void JP_C_16(u_int16_t& PC);
// No correspondence
void CALL_C_16(u_int16_t& PC);
// No correspondence
void SBC_A_8(u_int16_t& PC);
void RST_18(u_int16_t& PC);

// 0xE0 - 0xEF
void LDH_AD8_A(u_int16_t& PC);
void POP_HL(u_int16_t& PC);
void LDH_ADC_A(u_int16_t& PC);
// No correspondence
// No correspondence
void PUSH_HL(u_int16_t& PC);
void AND_A_8(u_int16_t& PC);
void RST_20(u_int16_t& PC);
void ADD_SP_8(u_int16_t& PC);
void JP_HL(u_int16_t& PC);
void LD_AD16_A(u_int16_t& PC);
// No correspondence
// No correspondence
// No correspondence
void XOR_A_8(u_int16_t& PC);
void RST_28(u_int16_t& PC);

// 0xF0 - 0xFF
void LDH_A_8(u_int16_t& PC);
void POP_AF(u_int16_t& PC);
void LDH_A_AD_C(u_int16_t& PC);
void DI(u_int16_t& PC);
// No correspondence
void PUSH_AF(u_int16_t& PC);
void OR_A_8(u_int16_t& PC);
void RST_30(u_int16_t& PC);
void LD_HL_SP8(u_int16_t& PC);
void LD_SP_HL(u_int16_t& PC);
void LD_A_16(u_int16_t& PC);
void EI(u_int16_t& PC);
// No correspondence
// No correspondence
void CP_A_8(u_int16_t& PC);
void RST_38(u_int16_t& PC);