#pragma once

#include <cstdint>
#include <vector>

std::vector<uint16_t> get_tile(uint16_t ID, std::vector<uint8_t>& RAM);
void draw_line(std::vector<uint8_t>& RAM, Image& screen);