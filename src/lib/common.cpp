#include "std.h"
#include "common.h"

#include <string> // to_string(const fs::path&) goes through std::string

#include "immintrin.h"
#include "meow-hash/meow_hash_x64_aesni.h"

// Default data folder path. Can be changed with -data-dir command line option.
static String g_data_dir = "./../data";

void error(const char* format, ...) {
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

bool fs_exists(const fs::path& path) {
    std::error_code ec;
    bool result = fs::exists(path, ec);
    ASSERT(!ec);
    return result;
}

bool fs_create_directories(const fs::path& path) {
    std::error_code ec;
    return fs::create_directories(path, ec);
}

bool fs_delete_directory(const fs::path& path) {
    std::error_code ec;
    return fs::remove_all(path, ec) != static_cast<std::uintmax_t>(-1);
}

bool fs_is_empty(const fs::path& path) {
    std::error_code ec;
    bool result = fs::is_empty(path, ec);
    ASSERT(!ec);
    return result;
}

bool fs_rename(const fs::path& old_path, const fs::path& new_path) {
    std::error_code ec;
    fs::rename(old_path, new_path, ec);
    return !ec;
}

String to_string(const fs::path& path)
{
    std::string s = path.string();
    return String(s.c_str(), (uint32_t)s.size());
}

void set_data_directory(const char* path)
{
    g_data_dir = path;
}

fs::path get_data_directory()
{
    return g_data_dir.data();
}

String to_lower(const char* s)
{
    uint32_t n = (uint32_t)strlen(s);
    char buffer[256];
    char* p = n <= sizeof(buffer) ? buffer : (char*)malloc(n);
    for (uint32_t i = 0; i < n; i++)
        p[i] = (char)tolower((unsigned char)s[i]);
    String result(p, n);
    if (p != buffer)
        free(p);
    return result;
}

String get_project_unique_name(const char* scene_path) {
    String file_name = to_lower(to_string(fs::path(scene_path).filename()).data());
    if (file_name.empty())
        error("Failed to extract filename from scene path: %s", scene_path);

    String path_lowercase = to_lower(scene_path);
    meow_u128 hash_128 = MeowHash(MeowDefaultSeed, path_lowercase.size(), (void*)path_lowercase.data());
    uint32_t hash_32 = MeowU32From(hash_128, 0);
    return string_printf("%08x-%s", hash_32, file_name.data());
}

String get_spirv_file(const char* spirv_base_name)
{
    return to_string(get_data_directory() / "spirv" / string_printf("%s.spv", spirv_base_name).data());
}

std::vector<uint8_t> read_binary_file(const char* file_path) {
    std::ifstream file(file_path, std::ios_base::in | std::ios_base::binary);
    if (!file)
        error("failed to open file: %s", file_path);

    // get file size
    file.seekg(0, std::ios_base::end);
    std::streampos file_size = file.tellg();
    file.seekg(0, std::ios_base::beg);

    if (file_size == std::streampos(-1) || !file)
        error("failed to read file stats: %s", file_path);

    // read file content
    std::vector<uint8_t> file_content(static_cast<size_t>(file_size));
    file.read(reinterpret_cast<char*>(file_content.data()), file_size);
    if (!file)
        error("failed to read file content: %s", file_path);

    return file_content;
}

String read_text_file(const char* file_path) {
    Scoped_File file = fopen(file_path, "rb");
    if (!file)
        error("failed to open file: %s", file_path);

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    if (file_size < 0)
        error("failed to read file stats: %s", file_path);

    char* buffer = (char*)malloc(file_size + 1);
    if (fread(buffer, 1, file_size, file) != (size_t)file_size)
        error("failed to read file content: %s", file_path);

    String content(buffer, (uint32_t)file_size);
    free(buffer);
    return content;
}

String get_extension(const char* file_path) {
    return to_lower(to_string(fs::path(file_path).extension()).data());
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

int64_t elapsed_milliseconds(Timestamp timestamp) {
    auto duration = std::chrono::steady_clock::now() - timestamp.t;
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    return static_cast<int64_t>(milliseconds);
}

int64_t elapsed_microseconds(Timestamp timestamp) {
    auto duration = std::chrono::steady_clock::now() - timestamp.t;
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    return static_cast<int64_t>(microseconds);
}

int64_t elapsed_nanoseconds(Timestamp timestamp) {
    auto duration = std::chrono::steady_clock::now() - timestamp.t;
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    return static_cast<int64_t>(nanoseconds);
}

float elapsed_seconds(Timestamp timestamp)
{
    return (float)((double)elapsed_nanoseconds(timestamp) / 1e9);
}

void enable_invalid_fp_exception() {
    _MM_SET_EXCEPTION_STATE(0); // reset current exception state
    _MM_SET_EXCEPTION_MASK(_MM_MASK_MASK & ~_MM_MASK_INVALID /*un-mask invalid fp exception bit*/);
}

void initialize_fp_state() {
#if ENABLE_INVALID_FP_EXCEPTION
    enable_invalid_fp_exception();
#endif
}
