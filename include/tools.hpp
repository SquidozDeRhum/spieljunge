#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <raylib.h>

#include "./types.hpp"

void loadROM(std::vector<u_int8_t>& RAM, std::string filename);
void loadBoot(std::vector<u_int8_t>& RAM, std::string filename);
void displayROM(std::vector<u_int8_t>& RAM);
void displayMemorySection(std::vector<u_int8_t>& RAM, int start, int end);
void flagsOutput(uint8_t F);

std::string R16_to_str(uint16_t R);
std::string R8_to_str(uint16_t R);

void CALL_Vector(uint16_t& PC, std::vector<uint8_t>& RAM, uint16_t& SP, uint16_t vector);
void doCPUStuff(Registers& registers, std::vector<uint8_t>& RAM, Image& screen);

void DMACopy(std::vector<uint8_t>& RAM);
void writeRAM(uint16_t address, uint8_t value, std::vector<uint8_t>& RAM);

// execute current instruction
void ECI(Registers& registers, std::vector<uint8_t>& RAM);
void EPR(Registers& registers, std::vector<uint8_t>& RAM);