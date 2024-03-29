#pragma once

#define ENABLE_ASSERT 1
#define ENABLE_PROFILING 1
#define ENABLE_INVALID_FP_EXCEPTION 1
#define ENABLE_PREFETCH 1

#if ENABLE_ASSERT
#define ASSERT(expression) if (expression) {} else __debugbreak()
#else
#define ASSERT(expression)
#endif

void error(const std::string& message);
void error(const char* format, ...);

namespace fs = std::filesystem;
bool fs_exists(const fs::path& path);
bool fs_create_directories(const fs::path& path); 
bool fs_delete_directory(const fs::path & path);
bool fs_is_empty(const fs::path& path);
bool fs_rename(const fs::path& old_path, const fs::path& new_path);

// The place where program's resources are located (spirv binaries) and also
// the program can write to this location if necessary (kdtree cache.
fs::path get_data_directory();

// Returns a name that can be used to create a directory to store additional/generated project data.
// The name is based on the hash of the scene's full path. So, for different project files that
// reference the same scene this function will return the same string.
//
// NOTE: if per project temp directories are needed then one option is to create project
// specific subdirectories inside temp scene directory - in this case we can share 
// scene's additional data between multiple projects.
std::string get_project_unique_name(const std::string & scene_path);

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
float elapsed_seconds(Timestamp timestamp);

#if ENABLE_PROFILING
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
#define REPORT_FUNCTION_TIME()
#define REPORT_SCOPE_TIME(...)
#endif

#if 0
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

inline uint32_t count_leading_zeros(uint32_t k) {
#ifdef _MSC_VER
    // LZCNT instruction is exactly what we need but it's not supported on pre-Haswell Intel CPUs.
    // Example implementation with LZCNT instruction:
    // return __lzcnt(k);

    unsigned long index;
    unsigned char result = _BitScanReverse(&index, k);
    return result ? (31 - (uint32_t)index) : 32;
#else
    uint32_t n = 0;
    while (k > 0) {
        k >>= 1;
        n++;
    }
    return 32 - n;
#endif
}

inline uint32_t most_significant_bit_index(uint32_t k) {
#ifdef _MSC_VER
    unsigned long index;
    unsigned char result = _BitScanReverse(&index, k);
    return result ? (uint32_t)index : 32;
#else
#error most_significant_bit_index is not implemented for non-MSVC compiler
#endif
}

inline float to_MB(uint64_t bytes) {
    return float(double(bytes) / (1024.0 * 1024.0));
}

// These functions control per-thread state.
void enable_invalid_fp_exception();
void initialize_fp_state();

struct Scoped_File {
    FILE* f = nullptr;
    Scoped_File(FILE* f) : f(f) {}
    ~Scoped_File() { if (f != nullptr) fclose(f); }
    operator FILE* () { return f; }
};

inline void prefetch(const void* ptr) {
#if ENABLE_PREFETCH
    _mm_prefetch((const char*)ptr, _MM_HINT_T0);
#endif
}

inline void* allocate_aligned_memory(size_t size, size_t alignment)
{
#ifdef _MSC_VER
    return _aligned_malloc(size, alignment);
#else
#error allocate_aligned_memory is not implemented
#endif
}

inline void free_aligned_memory(void* ptr)
{
#ifdef _MSC_VER
    _aligned_free(ptr);
#else
#error free_aligned_memory is not implemented
#endif
}

// Workaround for static_assert(false). It should be used like this: static_assert(dependent_false_v<T>)
template<typename>
inline constexpr bool dependent_false_v = false;
