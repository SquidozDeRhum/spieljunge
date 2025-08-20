#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

void loadROM(std::vector<u_int8_t>& RAM, std::string filename);
void loadBoot(std::vector<u_int8_t>& RAM, std::string filename);
void displayROM(std::vector<u_int8_t>& RAM);
void displayMemorySection(std::vector<u_int8_t>& RAM, int start, int end);
void flagsOutput(uint8_t F);

std::string R16_to_str(uint16_t R);
std::string R8_to_str(uint16_t R);

// execute current instruction
void ECI(uint8_t& A, uint8_t& F, uint8_t& B, uint8_t& C, uint8_t& D, uint8_t& E, uint8_t& H, uint8_t& L, uint16_t& SP, uint16_t& PC, std::vector<uint8_t>& RAM, int& cycles_counter);
void EPR(uint8_t& A, uint8_t& F, uint8_t& B, uint8_t& C, uint8_t& D, uint8_t& E, uint8_t& H, uint8_t& L, uint16_t& SP, uint16_t& PC, std::vector<uint8_t>& RAM, int& cycles_counter);