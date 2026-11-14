#pragma once

#include "minilib_platform.h"
#include <math.h>

#define ENABLE_PROFILING 1
#define ENABLE_INVALID_FP_EXCEPTION 1
#define ENABLE_PREFETCH 1

[[noreturn]] void error(const String& message);
[[noreturn]] void error(const char* format, ...);

// The place where program's resources are located (spirv binaries) and also
// the program can write to this location if necessary (kdtree cache).
void set_data_directory(const String& path);
String get_data_directory();

// Returns a name that can be used to create a directory to store additional/generated project data.
// The name is based on the hash of the scene's full path. So, for different project files that
// reference the same scene this function will return the same string.
//
// NOTE: if per project temp directories are needed then one option is to create project
// specific subdirectories inside temp scene directory - in this case we can share 
// scene's additional data between multiple projects.
String get_project_unique_name(const String& scene_path);

Array<uint8_t> load_spirv(const char* spirv_base_name);

double get_base_cpu_frequency_ghz();
double get_cpu_frequency_ghz();

#if ENABLE_PROFILING
struct Profile_Scope {
    const char* message = nullptr;
    Timestamp t;

    Profile_Scope(const char* message)
        : message(message) {}

    ~Profile_Scope() {
        printf("Profiler: %s %.2f ms\n", message, elapsed_nanoseconds(t) / 1e6f);
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

inline float srgb_encode(float f) {
    if (f <= 0.0031308f)
        return 12.92f * f;
    else
        return 1.055f * powf(f, 1.f/2.4f) - 0.055f;
}

inline float srgb_decode(float f) {
    if (f <= 0.04045f)
        return f / 12.92f;
    else
        return powf((f + 0.055f) / 1.055f, 2.4f);
}

uint32_t most_significant_bit_index(uint32_t k);

inline float to_MB(uint64_t bytes) {
    return float(double(bytes) / (1024.0 * 1024.0));
}

// These functions control per-thread state.
void initialize_fp_state();

// Workaround for static_assert(false). It should be used like this: static_assert(dependent_false_v<T>)
template<typename>
inline constexpr bool dependent_false_v = false;
