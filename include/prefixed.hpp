#pragma once

#include <iostream>
#include <cstdint>
#include <vector>

void RLC_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void RLC_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void RRC_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void RRC_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void RL_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void RL_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void RR_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void RR_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void SLA_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void SLA_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void SRA_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void SRA_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void SWAP_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void SWAP_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void SRL_R(uint16_t& PC, uint8_t& R, uint8_t& F);
void SRL_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F);

void BIT_N_R(uint16_t& PC, uint8_t& R, uint8_t& F, int N);
void BIT_N_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, uint8_t& F, int N);

void RES_N_R(uint16_t& PC, uint8_t& R, int N);
void RES_N_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, int N);

void SET_N_R(uint16_t& PC, uint8_t& R, int N);
void SET_N_ADHL(u_int16_t& PC, std::vector<uint8_t>& RAM, uint8_t H, uint8_t L, int N);