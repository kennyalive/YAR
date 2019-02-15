#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#define ASSERT(expression) if (!(expression)) __debugbreak()

namespace fs = std::filesystem;

constexpr float Pi = 3.14159265f;
constexpr float Pi_Inv = 1.f / Pi;

constexpr float Infinity = std::numeric_limits<float>::infinity();

template<class T, size_t N>
constexpr size_t array_size(T (&)[N]) {
    return N;
}

template <class T, uint32_t N>
constexpr uint32_t array_size32(T(&)[N]) {
    return N;
}

inline float radians(float degrees) {
    constexpr float deg_2_rad = Pi / 180.f;
    return degrees * deg_2_rad;
}

inline float degrees(float radians) {
    constexpr float rad_2_deg = 180.f / Pi;
    return radians * rad_2_deg;
}

inline float lerp(float t, float a, float b) {
    return a + (b - a)*t;
}

void error(const std::string& message);

bool fs_exists(const fs::path& path);
bool fs_remove_all(const fs::path& path);
bool fs_create_directory(const fs::path& path);

fs::path get_data_dir_path();
std::string get_directory(const std::string& path);
std::string get_resource_path(const std::string& resource_relative_path);
std::vector<uint8_t> read_binary_file(const std::string& file_name);

struct Timestamp {
    Timestamp() : t(std::chrono::steady_clock::now()) {}
    std::chrono::time_point<std::chrono::steady_clock> t;
};

double get_base_cpu_frequency_ghz();
double get_cpu_frequency_ghz();

int64_t elapsed_milliseconds(Timestamp timestamp);
int64_t elapsed_microseconds(Timestamp timestamp);
int64_t elapsed_nanoseconds(Timestamp timestamp);

// Boost hash combine.
template <typename T>
inline void hash_combine(std::size_t& seed, T value) {
    std::hash<T> hasher;
    seed ^= hasher(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template <typename T>
T& index_array_with_stride(T* base_ptr, size_t stride, size_t index) {
    return *(T*)((uint8_t*)base_ptr + stride * index);
}

template <typename T>
const T& index_array_with_stride(const T* base_ptr, size_t stride, size_t index) {
    return *(const T*)((uint8_t*)base_ptr + stride * index);
}

inline float srgb_encode(float f) {
    if (f <= 0.0031308f)
        return 12.92f * f;
    else
        return 1.055f * std::pow(f, 1.f/2.4f) - 0.055f;
}

inline uint32_t round_up(uint32_t k, uint32_t alignment) {
    return (k + alignment - 1) & ~(alignment - 1);
}

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
