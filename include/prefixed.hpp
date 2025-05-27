#pragma once

#include <iostream>

/*
* Here are shown all prefixed opcode
* They seems to all be bit manipulation instructions
*/

// 0x00 - 0x0F
void RLC_B(u_int16_t& PC);
void RLC_C(u_int16_t& PC);
void RLC_D(u_int16_t& PC);
void RLC_E(u_int16_t& PC);
void RLC_H(u_int16_t& PC);
void RLC_L(u_int16_t& PC);
void RLC_ADHL(u_int16_t& PC);
void RLC_A(u_int16_t& PC);
void RRC_B(u_int16_t& PC);
void RRC_C(u_int16_t& PC);
void RRC_D(u_int16_t& PC);
void RRC_E(u_int16_t& PC);
void RRC_H(u_int16_t& PC);
void RRC_L(u_int16_t& PC);
void RRC_ADHL(u_int16_t& PC);
void RRC_A(u_int16_t& PC);

// 0x10 - 0x1F
void RL_B(u_int16_t& PC);
void RL_C(u_int16_t& PC);
void RL_D(u_int16_t& PC);
void RL_E(u_int16_t& PC);
void RL_H(u_int16_t& PC);
void RL_L(u_int16_t& PC);
void RL_ADHL(u_int16_t& PC);
void RL_A(u_int16_t& PC);
void RR_B(u_int16_t& PC);
void RR_C(u_int16_t& PC);
void RR_D(u_int16_t& PC);
void RR_E(u_int16_t& PC);
void RR_H(u_int16_t& PC);
void RR_L(u_int16_t& PC);
void RR_ADHL(u_int16_t& PC);
void RR_A(u_int16_t& PC);

// 0x20 - 0x2F
void SLA_B(u_int16_t& PC);
void SLA_C(u_int16_t& PC);
void SLA_D(u_int16_t& PC);
void SLA_E(u_int16_t& PC);
void SLA_H(u_int16_t& PC);
void SLA_L(u_int16_t& PC);
void SLA_ADHL(u_int16_t& PC);
void SLA_A(u_int16_t& PC);
void SRA_B(u_int16_t& PC);
void SRA_C(u_int16_t& PC);
void SRA_D(u_int16_t& PC);
void SRA_E(u_int16_t& PC);
void SRA_H(u_int16_t& PC);
void SRA_L(u_int16_t& PC);
void SRA_ADHL(u_int16_t& PC);
void SRA_A(u_int16_t& PC);

// 0x30 - 0x3F
void SWAP_B(u_int16_t& PC);
void SWAP_C(u_int16_t& PC);
void SWAP_D(u_int16_t& PC);
void SWAP_E(u_int16_t& PC);
void SWAP_H(u_int16_t& PC);
void SWAP_L(u_int16_t& PC);
void SWAP_ADHL(u_int16_t& PC);
void SWAP_A(u_int16_t& PC);
void SRL_B(u_int16_t& PC);
void SRL_C(u_int16_t& PC);
void SRL_D(u_int16_t& PC);
void SRL_E(u_int16_t& PC);
void SRL_H(u_int16_t& PC);
void SRL_L(u_int16_t& PC);
void SRL_ADHL(u_int16_t& PC);
void SRL_A(u_int16_t& PC);

// 0x40 - 0x4F
void BIT_0_B(u_int16_t& PC);
void BIT_0_C(u_int16_t& PC);
void BIT_0_D(u_int16_t& PC);
void BIT_0_E(u_int16_t& PC);
void BIT_0_H(u_int16_t& PC);
void BIT_0_L(u_int16_t& PC);
void BIT_0_ADHL(u_int16_t& PC);
void BIT_0_AD(u_int16_t& PC);
void BIT_1_B(u_int16_t& PC);
void BIT_1_C(u_int16_t& PC);
void BIT_1_D(u_int16_t& PC);
void BIT_1_E(u_int16_t& PC);
void BIT_1_H(u_int16_t& PC);
void BIT_1_L(u_int16_t& PC);
void BIT_1_ADHL(u_int16_t& PC);
void BIT_1_A(u_int16_t& PC);

// 0x50 - 0x5F
void BIT_2_B(u_int16_t& PC);
void BIT_2_C(u_int16_t& PC);
void BIT_2_D(u_int16_t& PC);
void BIT_2_E(u_int16_t& PC);
void BIT_2_H(u_int16_t& PC);
void BIT_2_L(u_int16_t& PC);
void BIT_2_ADHL(u_int16_t& PC);
void BIT_2_AD(u_int16_t& PC);
void BIT_3_B(u_int16_t& PC);
void BIT_3_C(u_int16_t& PC);
void BIT_3_D(u_int16_t& PC);
void BIT_3_E(u_int16_t& PC);
void BIT_3_H(u_int16_t& PC);
void BIT_3_L(u_int16_t& PC);
void BIT_3_ADHL(u_int16_t& PC);
void BIT_3_A(u_int16_t& PC);

// 0x60 - 0x6F
void BIT_4_B(u_int16_t& PC);
void BIT_4_C(u_int16_t& PC);
void BIT_4_D(u_int16_t& PC);
void BIT_4_E(u_int16_t& PC);
void BIT_4_H(u_int16_t& PC);
void BIT_4_L(u_int16_t& PC);
void BIT_4_ADHL(u_int16_t& PC);
void BIT_4_AD(u_int16_t& PC);
void BIT_5_B(u_int16_t& PC);
void BIT_5_C(u_int16_t& PC);
void BIT_5_D(u_int16_t& PC);
void BIT_5_E(u_int16_t& PC);
void BIT_5_H(u_int16_t& PC);
void BIT_5_L(u_int16_t& PC);
void BIT_5_ADHL(u_int16_t& PC);
void BIT_5_A(u_int16_t& PC);

// 0x70 - 0x7F
void BIT_6_B(u_int16_t& PC);
void BIT_6_C(u_int16_t& PC);
void BIT_6_D(u_int16_t& PC);
void BIT_6_E(u_int16_t& PC);
void BIT_6_H(u_int16_t& PC);
void BIT_6_L(u_int16_t& PC);
void BIT_6_ADHL(u_int16_t& PC);
void BIT_6_AD(u_int16_t& PC);
void BIT_7_B(u_int16_t& PC);
void BIT_7_C(u_int16_t& PC);
void BIT_7_D(u_int16_t& PC);
void BIT_7_E(u_int16_t& PC);
void BIT_7_H(u_int16_t& PC);
void BIT_7_L(u_int16_t& PC);
void BIT_7_ADHL(u_int16_t& PC);
void BIT_7_A(u_int16_t& PC);

// 0x80 - 0x8F
void RES_0_B(u_int16_t& PC);
void RES_0_C(u_int16_t& PC);
void RES_0_D(u_int16_t& PC);
void RES_0_E(u_int16_t& PC);
void RES_0_H(u_int16_t& PC);
void RES_0_L(u_int16_t& PC);
void RES_0_ADHL(u_int16_t& PC);
void RES_0_AD(u_int16_t& PC);
void RES_1_B(u_int16_t& PC);
void RES_1_C(u_int16_t& PC);
void RES_1_D(u_int16_t& PC);
void RES_1_E(u_int16_t& PC);
void RES_1_H(u_int16_t& PC);
void RES_1_L(u_int16_t& PC);
void RES_1_ADHL(u_int16_t& PC);
void RES_1_A(u_int16_t& PC);

// 0x90 - 0x9F
void RES_2_B(u_int16_t& PC);
void RES_2_C(u_int16_t& PC);
void RES_2_D(u_int16_t& PC);
void RES_2_E(u_int16_t& PC);
void RES_2_H(u_int16_t& PC);
void RES_2_L(u_int16_t& PC);
void RES_2_ADHL(u_int16_t& PC);
void RES_2_AD(u_int16_t& PC);
void RES_3_B(u_int16_t& PC);
void RES_3_C(u_int16_t& PC);
void RES_3_D(u_int16_t& PC);
void RES_3_E(u_int16_t& PC);
void RES_3_H(u_int16_t& PC);
void RES_3_L(u_int16_t& PC);
void RES_3_ADHL(u_int16_t& PC);
void RES_3_A(u_int16_t& PC);

// 0xA0 - 0xAF
void RES_4_B(u_int16_t& PC);
void RES_4_C(u_int16_t& PC);
void RES_4_D(u_int16_t& PC);
void RES_4_E(u_int16_t& PC);
void RES_4_H(u_int16_t& PC);
void RES_4_L(u_int16_t& PC);
void RES_4_ADHL(u_int16_t& PC);
void RES_4_AD(u_int16_t& PC);
void RES_5_B(u_int16_t& PC);
void RES_5_C(u_int16_t& PC);
void RES_5_D(u_int16_t& PC);
void RES_5_E(u_int16_t& PC);
void RES_5_H(u_int16_t& PC);
void RES_5_L(u_int16_t& PC);
void RES_5_ADHL(u_int16_t& PC);
void RES_5_A(u_int16_t& PC);

// 0xB0 - 0xBF
void RES_6_B(u_int16_t& PC);
void RES_6_C(u_int16_t& PC);
void RES_6_D(u_int16_t& PC);
void RES_6_E(u_int16_t& PC);
void RES_6_H(u_int16_t& PC);
void RES_6_L(u_int16_t& PC);
void RES_6_ADHL(u_int16_t& PC);
void RES_6_AD(u_int16_t& PC);
void RES_7_B(u_int16_t& PC);
void RES_7_C(u_int16_t& PC);
void RES_7_D(u_int16_t& PC);
void RES_7_E(u_int16_t& PC);
void RES_7_H(u_int16_t& PC);
void RES_7_L(u_int16_t& PC);
void RES_7_ADHL(u_int16_t& PC);
void RES_7_A(u_int16_t& PC);

// 0xC0 - 0xCF
void SET_0_B(u_int16_t& PC);
void SET_0_C(u_int16_t& PC);
void SET_0_D(u_int16_t& PC);
void SET_0_E(u_int16_t& PC);
void SET_0_H(u_int16_t& PC);
void SET_0_L(u_int16_t& PC);
void SET_0_ADHL(u_int16_t& PC);
void SET_0_AD(u_int16_t& PC);
void SET_1_B(u_int16_t& PC);
void SET_1_C(u_int16_t& PC);
void SET_1_D(u_int16_t& PC);
void SET_1_E(u_int16_t& PC);
void SET_1_H(u_int16_t& PC);
void SET_1_L(u_int16_t& PC);
void SET_1_ADHL(u_int16_t& PC);
void SET_1_A(u_int16_t& PC);

// 0xD0 - 0xDF
void SET_2_B(u_int16_t& PC);
void SET_2_C(u_int16_t& PC);
void SET_2_D(u_int16_t& PC);
void SET_2_E(u_int16_t& PC);
void SET_2_H(u_int16_t& PC);
void SET_2_L(u_int16_t& PC);
void SET_2_ADHL(u_int16_t& PC);
void SET_2_AD(u_int16_t& PC);
void SET_3_B(u_int16_t& PC);
void SET_3_C(u_int16_t& PC);
void SET_3_D(u_int16_t& PC);
void SET_3_E(u_int16_t& PC);
void SET_3_H(u_int16_t& PC);
void SET_3_L(u_int16_t& PC);
void SET_3_ADHL(u_int16_t& PC);
void SET_3_A(u_int16_t& PC);

// 0xE0 - 0xEF
void SET_4_B(u_int16_t& PC);
void SET_4_C(u_int16_t& PC);
void SET_4_D(u_int16_t& PC);
void SET_4_E(u_int16_t& PC);
void SET_4_H(u_int16_t& PC);
void SET_4_L(u_int16_t& PC);
void SET_4_ADHL(u_int16_t& PC);
void SET_4_AD(u_int16_t& PC);
void SET_5_B(u_int16_t& PC);
void SET_5_C(u_int16_t& PC);
void SET_5_D(u_int16_t& PC);
void SET_5_E(u_int16_t& PC);
void SET_5_H(u_int16_t& PC);
void SET_5_L(u_int16_t& PC);
void SET_5_ADHL(u_int16_t& PC);
void SET_5_A(u_int16_t& PC);

// 0xF0 - 0xFF
void SET_6_B(u_int16_t& PC);
void SET_6_C(u_int16_t& PC);
void SET_6_D(u_int16_t& PC);
void SET_6_E(u_int16_t& PC);
void SET_6_H(u_int16_t& PC);
void SET_6_L(u_int16_t& PC);
void SET_6_ADHL(u_int16_t& PC);
void SET_6_AD(u_int16_t& PC);
void SET_7_B(u_int16_t& PC);
void SET_7_C(u_int16_t& PC);
void SET_7_D(u_int16_t& PC);
void SET_7_E(u_int16_t& PC);
void SET_7_H(u_int16_t& PC);
void SET_7_L(u_int16_t& PC);
void SET_7_ADHL(u_int16_t& PC);
void SET_7_A(u_int16_t& PC);