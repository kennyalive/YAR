#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

constexpr float Pi = 3.14159265f;

constexpr float Infinity = std::numeric_limits<float>::infinity();

inline void error(const std::string& message) {
    printf("error: %s\n", message.c_str());
    exit(1);
}

struct Timestamp {
    Timestamp() : t(std::chrono::steady_clock::now()) {}
    const std::chrono::time_point<std::chrono::steady_clock> t;
};

double get_base_cpu_frequency_ghz();
double get_cpu_frequency_ghz();

int64_t elapsed_milliseconds(Timestamp timestamp);
int64_t elapsed_nanoseconds(Timestamp timestamp);

inline float Radians(float degrees) {
    constexpr float deg_2_rad = Pi / 180.f;
    return degrees * deg_2_rad;
}

inline float Degrees(float radians) {
    constexpr float rad_2_deg = 180.f / Pi;
    return radians * rad_2_deg;
}
