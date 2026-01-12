#ifndef RPM_CONVERTER_HPP
#define RPM_CONVERTER_HPP

#define MAX_BIT 8191
#define MAX_RPM 9800

#include <iostream>
#include <cstdint>

inline double bit_to_double_rpm(int16_t bit_value);

#endif