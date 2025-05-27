#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

void loadROM(std::vector<u_int8_t>& RAM, std::string filename);
void displayROM(std::vector<u_int8_t>& RAM);