#include "std.h"
#include "common.h"
#include "minilib.h"
#include "path.h"

#include <stdarg.h>
#include <stdlib.h>
#include "immintrin.h"
#include "meow-hash/meow_hash_x64_aesni.h"

// Default data folder path. Can be changed with -data-dir command line option.
static String g_data_dir = "./../data";

[[noreturn]] void error(const String& message) {
    printf("\nError: %s\n", message.data());
#ifdef _WIN32
    __debugbreak();
#endif
    exit(1);
}

[[noreturn]] void error(const char* format, ...) {
    printf("\nError: ");
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
#ifdef _WIN32
    __debugbreak();
#endif
    exit(1);
}

void set_data_directory(const String& path)
{
    g_data_dir = path;
}

String get_data_directory()
{
    return g_data_dir;
}

String get_project_unique_name(const String& scene_path) {
    String file_name = string_to_lower(path_filename(scene_path));
    if (file_name.empty())
        error("Failed to extract filename from scene path: %s", scene_path.data());

    String path_lowercase = string_to_lower(scene_path);
    meow_u128 hash_128 = MeowHash(MeowDefaultSeed, path_lowercase.size(), (void*)path_lowercase.data());
    uint32_t hash_32 = MeowU32From(hash_128, 0);

    return string_concat(string_printf("%08x", hash_32), "-", file_name);
}

String get_spirv_file(const char* spirv_base_name)
{
    String path = path_join(path_join(get_data_directory(), "spirv"), string_concat(spirv_base_name, ".spv"));
    return path;
}

double get_base_cpu_frequency_ghz() {
    auto rdtsc_start = __rdtsc();
    Timestamp t;
    while (elapsed_milliseconds(t) < 1000) {}
    auto rdtsc_end = __rdtsc();
    double frequency = ((rdtsc_end - rdtsc_start) / 1'000'000) / 1000.0;
    return frequency;
}

double get_cpu_frequency_ghz() {
#ifdef CPU_FREQ_GHZ
    return CPU_FREQ_GHZ;
#else
    return get_base_cpu_frequency_ghz();
#endif
}

void initialize_fp_state() {
#if ENABLE_INVALID_FP_EXCEPTION
    enable_invalid_fp_exception();
#endif
}
