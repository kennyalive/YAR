#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

template <typename T, size_t N>
constexpr uint32_t array_length(T(&)[N]) {
    return N;
}

void error(const std::string& message);
std::vector<uint8_t> read_binary_file(const std::string& file_name);

struct Timestamp {
    Timestamp() : t(std::chrono::steady_clock::now()) {}
    std::chrono::time_point<std::chrono::steady_clock> t;
};

int64_t elapsed_milliseconds(Timestamp timestamp);
int64_t elapsed_nanoseconds(Timestamp timestamp);

double get_base_cpu_frequency_ghz();

#if 1
#define START_TIMER { Timestamp t;
#define STOP_TIMER(message) \
	auto d = elapsed_nanoseconds(t); \
	static Timestamp t0; \
	if (elapsed_milliseconds(t0) > 1000) { \
		t0 = Timestamp(); \
		printf(message ## " time = %lld  microseconds\n", d / 1000); } }

#else
#define START_TIMER
#define STOP_TIMER(...)
#endif
