#include "platform.h"
#include <chrono>

bool fs_load(String_View path, Array<uint8_t>& bytes)
{
    Scoped_File file = fs_open(path, "rb");
    if (!file) {
        return false;
    }
    size_t size{};
    if (!fs_file_size(file, size)) {
        return false;
    }
    Array<uint8_t> content;
    content.resize_uninitialized(size);
    if (!file.read(content.data(), size)) {
        return false;
    }
    bytes = rvalue(content);
    return true;
}

bool fs_save(String_View path, const void* data, size_t size)
{
    ASSERT(data || !size);
    Scoped_File file = fs_open(path, "wb");
    if (!file) {
        return false;
    }
    if (!file.write(data, size)) {
        return false;
    }
    return file.close();
}

bool fs_load_text(String_View path, String& text)
{
    Array<uint8_t> content;
    if (!fs_load(path, content)) {
        return false;
    }
    text = String(reinterpret_cast<const char*>(content.data()), content.size());
    return true;
}

bool fs_save_text(String_View path, String_View text)
{
    return fs_save(path, text.data, text.size);
}

Timestamp::Timestamp()
    : nanoseconds(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count())
{}

int64_t elapsed_nanoseconds(Timestamp start)
{
    return Timestamp().nanoseconds - start.nanoseconds;
}

int64_t elapsed_milliseconds(Timestamp start)
{
    return elapsed_nanoseconds(start) / 1'000'000;
}

float elapsed_seconds(Timestamp start)
{
    return float(double(elapsed_nanoseconds(start)) / 1e9);
}
