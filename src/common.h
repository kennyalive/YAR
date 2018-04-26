#pragma once

#include <algorithm>
#include <chrono>
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
