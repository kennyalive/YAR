#pragma once

#define ASSERT(expression) if (!(expression)) __debugbreak()

#define ENABLE_PROFILING 1

constexpr float Pi = 3.14159265f;
constexpr float Pi2 = 6.2831853f;
constexpr float Pi_Over_2 = 1.57079632f;
constexpr float Pi_Inv = 1.f / Pi;
constexpr float Pi2_Inv = 1.f / Pi2;
constexpr float Infinity = std::numeric_limits<float>::infinity();
constexpr float One_Minus_Epsilon = 0x1.fffffep-1;

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

// Returns extension in lower case in the form ".ext".
std::string get_extension(const std::string& file_path);

struct Timestamp {
    Timestamp() : t(std::chrono::steady_clock::now()) {}
    std::chrono::time_point<std::chrono::steady_clock> t;
};

double get_base_cpu_frequency_ghz();
double get_cpu_frequency_ghz();

int64_t elapsed_milliseconds(Timestamp timestamp);
int64_t elapsed_microseconds(Timestamp timestamp);
int64_t elapsed_nanoseconds(Timestamp timestamp);

#if ENABLE_PROFILING
#define START_TIMER { Timestamp t;
#define STOP_TIMER(message) \
	auto d = elapsed_nanoseconds(t); \
	static Timestamp t0; \
	if (elapsed_milliseconds(t0) > 1000) { \
		t0 = Timestamp(); \
		printf(message ## " time = %lld  microseconds\n", d / 1000); } }

struct Profile_Scope {
    const char* message = nullptr;
    Timestamp t;

    Profile_Scope(const char* message)
        : message(message) {}

    ~Profile_Scope() {
        printf("Profiler: %s %.2f ms\n", message, elapsed_microseconds(t) / 1000.f);
    }
};

#define REPORT_FUNCTION_TIME() Profile_Scope function_time_reporter(__FUNCTION__);
#define REPORT_SCOPE_TIME(message) Profile_Scope function_time_reporter(message);

#else
#define START_TIMER
#define STOP_TIMER(...)
REPORT_FUNCTION_TIME()
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

inline float srgb_decode(float f) {
    if (f <= 0.04045f)
        return f / 12.92f;
    else
        return std::pow((f + 0.055f) / 1.055f, 2.4f);
}

template <typename T>
inline T round_up(T k, T alignment) {
    return (k + alignment - 1) & ~(alignment - 1);
}

inline uint32_t count_leading_zeros(uint32_t k) {
#ifdef _MSC_VER
    return __lzcnt(k);
#else
    uint32_t n = 0;
    while (k > 0) {
        k >>= 1;
        n++;
    }
    return 32 - n;
#endif
}

inline uint32_t log2_int(uint32_t k) {
    ASSERT(k > 0);
    return 31 - count_leading_zeros(k);
}

inline uint32_t round_up_to_power_of_2(uint32_t k) {
    ASSERT(k > 0);
    k--;
    k |= k >> 1;
    k |= k >> 2;
    k |= k >> 4;
    k |= k >> 8;
    k |= k >> 16;
    k++;
    return k;
}

inline bool is_power_of_2(uint32_t k) {
    return k != 0 && (k & (k - 1)) == 0;
}
