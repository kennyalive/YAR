#include "common.h"
#include <fstream>

#ifdef _WIN32
#include <intrin.h>
#endif

void error(const std::string& message) {
    printf("error: %s\n", message.c_str());
    exit(1);
}

std::vector<uint8_t> read_binary_file(const std::string& file_name) {
      std::ifstream file(file_name, std::ios_base::in | std::ios_base::binary);
      if (!file)
        error("failed to open file: " + file_name);

      // get file size
      file.seekg(0, std::ios_base::end);
      std::streampos file_size = file.tellg();
      file.seekg(0, std::ios_base::beg);

      if (file_size == std::streampos(-1) || !file)
        error("failed to read file stats: " + file_name);

      // read file content
      std::vector<uint8_t> file_content(static_cast<size_t>(file_size));
      file.read(reinterpret_cast<char*>(file_content.data()), file_size);
      if (!file)
        error("failed to read file content: " + file_name);

      return file_content;
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

int64_t elapsed_nanoseconds(Timestamp timestamp) {
    auto duration = std::chrono::steady_clock::now() - timestamp.t;
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    return static_cast<int64_t>(nanoseconds);
}
