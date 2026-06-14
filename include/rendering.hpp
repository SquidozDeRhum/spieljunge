#pragma once

#include <cstdint>
#include <vector>

uint16_t getTileLine(uint8_t ID, uint8_t lineNumber, bool object, std::vector<uint8_t>& RAM);
void draw_line(std::vector<uint8_t>& RAM, Image& screen);