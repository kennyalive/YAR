#include "minilib.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

//
// String
//
static const char* heap_copy(const char* s, uint32_t n)
{
    char* p = new char[(size_t)n + 1];
    memcpy(p, s, n);
    p[n] = 0;
    return p;
}

String::String(const char* s)
    : String(s, (uint32_t)strlen(s))
{}

String::String(const char* s, uint32_t n)
{
    if (n <= max_small) {
        if (n)
            memcpy(storage.small, s, n);
        storage.small[n] = 0;
        storage.small[max_small] = char(max_small - n); // the terminator itself when n == max_small
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
    storage = other.storage; // takes over the heap pointer or the small characters
    other.storage.small[0] = 0;
    other.storage.small[max_small] = char(max_small);
}

String::~String()
{
    if (!is_small())
        delete[] storage.heap.chars;
}

void String::swap(String& other) noexcept
{
    Storage t = storage;
    storage = other.storage;
    other.storage = t;
}

bool operator==(const String& a, const String& b)
{
    return a.size() == b.size() && memcmp(a.data(), b.data(), a.size()) == 0;
}

bool operator==(const String& a, const char* b)
{
    return a.size() == strlen(b) && memcmp(a.data(), b, a.size()) == 0;
}

bool operator<(const String& a, const String& b)
{
    uint32_t n = a.size() < b.size() ? a.size() : b.size();
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
    if (n < 0)
        return String();
    if (n < (int)sizeof(buffer))
        return String(buffer, (uint32_t)n);

    // Longer than the stack buffer: format again into a heap buffer.
    char* big = new char[(size_t)n + 1];
    va_start(args, format);
    vsnprintf(big, (size_t)n + 1, format, args);
    va_end(args);
    String result(big, (uint32_t)n);
    delete[] big;
    return result;
}
