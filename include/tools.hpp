#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include "./types.hpp"

void loadROM(std::vector<u_int8_t>& RAM, std::string filename);
void loadBoot(std::vector<u_int8_t>& RAM, std::string filename);
void displayROM(std::vector<u_int8_t>& RAM);
void displayMemorySection(std::vector<u_int8_t>& RAM, int start, int end);
void flagsOutput(uint8_t F);

std::string R16_to_str(uint16_t R);
std::string R8_to_str(uint16_t R);

// execute current instruction
void ECI(Registers& registers, std::vector<uint8_t>& RAM);
void EPR(Registers& registers, std::vector<uint8_t>& RAM);