#pragma once

#define ASSERT(expression) if (!(expression)) __debugbreak()

constexpr float Pi = 3.14159265f;
constexpr float Pi_Inv = 1.f / Pi;
constexpr float Infinity = std::numeric_limits<float>::infinity();

void error(const std::string& message);
void error(const char* format, ...);

namespace fs = std::filesystem;
bool fs_exists(const fs::path& path);
bool fs_create_directories(const fs::path& path); 

// The place where program's resources are located (spirv binaries) and also
// the program can write to this location if necessary (kdtree cache.
fs::path get_data_directory();

std::vector<uint8_t> read_binary_file(const std::string& file_path);
std::string read_text_file(const std::string& file_path);

struct Timestamp {
    Timestamp() : t(std::chrono::steady_clock::now()) {}
    std::chrono::time_point<std::chrono::steady_clock> t;
};

double get_base_cpu_frequency_ghz();
double get_cpu_frequency_ghz();

int64_t elapsed_milliseconds(Timestamp timestamp);
int64_t elapsed_microseconds(Timestamp timestamp);
int64_t elapsed_nanoseconds(Timestamp timestamp);

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

// http://www.reedbeta.com/blog/python-like-enumerate-in-cpp17/
template <typename T,
    typename TIter = decltype(std::begin(std::declval<T>())),
    typename = decltype(std::end(std::declval<T>()))>
    constexpr auto enumerate(T && iterable)
{
    struct iterator
    {
        size_t i;
        TIter iter;
        bool operator != (const iterator & other) const { return iter != other.iter; }
        void operator ++ () { ++i; ++iter; }
        auto operator * () const { return std::tie(i, *iter); }
    };
    struct iterable_wrapper
    {
        T iterable;
        auto begin() { return iterator{ 0, std::begin(iterable) }; }
        auto end() { return iterator{ 0, std::end(iterable) }; }
    };
    return iterable_wrapper{ std::forward<T>(iterable) };
}

inline constexpr float radians(float degrees) {
    constexpr float deg_2_rad = Pi / 180.f;
    return degrees * deg_2_rad;
}

inline constexpr float degrees(float radians) {
    constexpr float rad_2_deg = 180.f / Pi;
    return radians * rad_2_deg;
}

inline float lerp(float t, float a, float b) {
    return a + (b - a)*t;
}

inline std::string to_lower(std::string s) {
    for (char& c : s)
        c = tolower(c);
    return s;
}

// Boost hash combine.
template <typename T>
inline void hash_combine(std::size_t& seed, T value) {
    std::hash<T> hasher;
    seed ^= hasher(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

inline float srgb_encode(float f) {
    if (f <= 0.0031308f)
        return 12.92f * f;
    else
        return 1.055f * std::pow(f, 1.f/2.4f) - 0.055f;
}

template <typename T>
inline T round_up(T k, T alignment) {
    return (k + alignment - 1) & ~(alignment - 1);
}
