#include "minilib.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

//
// String
//
static const char* heap_copy(const char* s, size_t n)
{
    char* p = new char[n + 1];
    memcpy(p, s, n);
    p[n] = 0;
    return p;
}

String::String(const char* s) : String(s, strlen(s)) {}

String::String(const char* s, size_t n)
{
    if (n <= max_small) {
        if (n != 0) {
            memcpy(storage.small, s, n);
        }
        storage.small[n] = 0;
        // Write remaining small storage space.
        // It is the terminator itself when n == max_small
        storage.small[max_small] = char(max_small - n);
    }
    else {
        storage.heap.chars = heap_copy(s, n);
        storage.heap.count = n;
        storage.heap.tag = heap_tag;
    }
}

String::String(const String& other)
{
    if (other.is_small()) {
        storage = other.storage;
    }
    else {
        storage.heap.chars = heap_copy(other.storage.heap.chars, other.storage.heap.count);
        storage.heap.count = other.storage.heap.count;
        storage.heap.tag = heap_tag;
    }
}

String::String(String&& other) noexcept
{
    storage = other.storage;
    other.storage.small[0] = 0;
    other.storage.small[max_small] = char(max_small);
}

String& String::operator=(const String& other)
{
    if (this == &other) {
        return *this;
    }
    if (!is_small()) {
        delete[] storage.heap.chars;
    }
    if (other.is_small()) {
        storage = other.storage;
    }
    else {
        storage.heap.chars = heap_copy(other.storage.heap.chars, other.storage.heap.count);
        storage.heap.count = other.storage.heap.count;
        storage.heap.tag = heap_tag;
    }
    return *this;
}

String& String::operator=(String&& other) noexcept
{
    if (this == &other) {
        return *this;
    }
    if (!is_small()) {
        delete[] storage.heap.chars;
    }
    storage = other.storage;
    other.storage.small[0] = 0;
    other.storage.small[max_small] = char(max_small);
    return *this;
}

String::~String()
{
    if (!is_small()) {
        delete[] storage.heap.chars;
    }
}

bool operator==(const String& a, const String& b)
{
    return a.size() == b.size() && memcmp(a.data(), b.data(), a.size()) == 0;
}

bool operator!=(const String& a, const String& b)
{
    return !(a == b);
}

bool operator==(const String& a, const char* b)
{
    return a.size() == strlen(b) && memcmp(a.data(), b, a.size()) == 0;
}

bool operator<(const String& a, const String& b)
{
    size_t n = a.size() < b.size() ? a.size() : b.size();
    int c = memcmp(a.data(), b.data(), n);
    return c < 0 || (c == 0 && a.size() < b.size());
}

String string_printf(const char* format, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (n < 0) {
        return {};
    }
    if (n < (int)sizeof(buffer)) {
        return String(buffer, (size_t)n);
    }
    char* alloced_buffer = new char[(size_t)n + 1];
    va_start(args, format);
    vsnprintf(alloced_buffer, (size_t)n + 1, format, args);
    va_end(args);
    String str;
    str.storage.heap.chars = alloced_buffer;
    str.storage.heap.count = n;
    str.storage.heap.tag = String::heap_tag;
    return str;
}
