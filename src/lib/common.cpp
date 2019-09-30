#include "std.h"
#include "common.h"
#include <cstdarg>

#ifdef _WIN32
#include <intrin.h>
#endif

// Default data folder path. Can be changed with --data-dir command line option.
std::string g_data_dir = "./../data";

void error(const std::string& message) {
    printf("\nError: %s\n", message.c_str());
#ifdef _WIN32
    __debugbreak();
#endif
    exit(1);
}

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
    return fs::exists(path, ec);
}

bool fs_remove_all(const fs::path& path) {
    std::error_code ec;
    return fs::remove_all(path, ec) != static_cast<std::uintmax_t>(-1);
}

bool fs_create_directory(const fs::path& path) {
    std::error_code ec;
    return fs::create_directory(path, ec);
}

bool fs_create_directories(const fs::path& path) {
    std::error_code ec;
    return fs::create_directories(path, ec);
}

static std::string join_paths(std::string path1, std::string path2) {
  if (!path1.empty() && (path1.back() == '/' || path1.back() == '\\'))
    path1 = path1.substr(0, path1.length() - 1);

  if (!path2.empty() && (path2[0] == '/' || path2[0] == '\\'))
    path2 = path2.substr(1, path2.length() - 1);

  return path1 + '/' + path2;
}

size_t get_last_slash_pos(const std::string& path) {
    size_t pos1 = path.rfind('/');
    size_t pos2 = path.rfind('\\');

    if (pos1 == std::string::npos && pos2 == std::string::npos)
        return std::string::npos;
    else if (pos1 == std::string::npos)
        return pos2;
    else if (pos2 == std::string::npos)
        return pos1;
    else
        return std::max(pos1, pos2);
}

fs::path get_data_directory() {
    return g_data_dir;
}

std::string get_directory(const std::string& path) {
    size_t slash_pos = get_last_slash_pos(path);
    return (slash_pos == std::string::npos) ? path : path.substr(0, slash_pos + 1);
}

std::string get_resource_path(const std::string& resource_relative_path) {
    return join_paths(g_data_dir, resource_relative_path);
}

std::vector<uint8_t> read_binary_file(const std::string& file_name) {
    std::string abs_path = get_resource_path(file_name);
    std::ifstream file(abs_path, std::ios_base::in | std::ios_base::binary);
    if (!file)
        error("failed to open file: " + abs_path);

    // get file size
    file.seekg(0, std::ios_base::end);
    std::streampos file_size = file.tellg();
    file.seekg(0, std::ios_base::beg);

    if (file_size == std::streampos(-1) || !file)
        error("failed to read file stats: " + abs_path);

    // read file content
    std::vector<uint8_t> file_content(static_cast<size_t>(file_size));
    file.read(reinterpret_cast<char*>(file_content.data()), file_size);
    if (!file)
        error("failed to read file content: " + abs_path);

    return file_content;
}

std::string read_text_file(const std::string& file_name) {
    std::string abs_path = get_resource_path(file_name);
    std::ifstream file(abs_path);
    if (!file)
        error("failed to open file: %s", abs_path.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
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
    //return get_base_cpu_frequency_ghz();
    return 4.5;
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
