#pragma once

#include <algorithm>
#include <limits>
#include <string>

constexpr float Infinity = std::numeric_limits<float>::infinity();

inline void error(const std::string& message) {
    printf("error: %s\n", message.c_str());
    exit(1);
}
