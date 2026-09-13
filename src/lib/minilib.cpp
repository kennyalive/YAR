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

String_View::String_View(const char* s) : data(s), size(strlen(s)) {}

bool operator==(String_View a, String_View b)
{
    if (a.size != b.size) {
        return false;
    }
    if (a.size == 0) {
        return true;
    }
    return memcmp(a.data, b.data, a.size) == 0;
}

bool operator!=(String_View a, String_View b)
{
    return !(a == b);
}

bool operator<(String_View a, String_View b)
{
    size_t n = a.size < b.size ? a.size : b.size;
    if (n != 0) {
        int c = memcmp(a.data, b.data, n);
        if (c != 0) {
            return c < 0;
        }
    }
    return a.size < b.size;
}

// Case is ignored for ascii letters only, other bytes must match exactly
bool equals_ignore_case(String_View a, String_View b)
{
    if (a.size != b.size) {
        return false;
    }
    for (size_t i = 0; i < a.size; i++) {
        char x = a.data[i];
        char y = b.data[i];
        if (x >= 'A' && x <= 'Z') x += 'a' - 'A';
        if (y >= 'A' && y <= 'Z') y += 'a' - 'A';
        if (x != y) {
            return false;
        }
    }
    return true;
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
    other.reset_storage();
}

String::~String()
{
    if (!is_small()) {
        delete[] storage.heap.chars;
    }
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
    other.reset_storage();
    return *this;
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

static String concat(const String_View* parts, size_t count)
{
    size_t total = 0;
    for (size_t i = 0; i < count; i++) {
        total += parts[i].size;
    }
    String result;
    char* p;
    if (total <= String::max_small) {
        p = result.storage.small;
        p[String::max_small] = char(String:: max_small - total);
    }
    else {
        p = new char[total + 1];
        result.storage.heap.chars = p;
        result.storage.heap.count = total;
        result.storage.heap.tag = String::heap_tag;
    }
    for (size_t i = 0; i < count; i++) {
        if (parts[i].size != 0) {
            memcpy(p, parts[i].data, parts[i].size);
            p += parts[i].size;
        }
    }
    *p = 0;
    return result;
}

String string_concat(String_View a, String_View b)
{
    String_View parts[] = {a, b};
    return concat(parts, 2);
}

String string_concat(String_View a, String_View b, String_View c)
{
    String_View parts[] = {a, b, c};
    return concat(parts, 3);
}

String string_concat(String_View a, String_View b, String_View c, String_View d)
{
    String_View parts[] = {a, b, c, d};
    return concat(parts, 4);
}

//
// Hashing
//
uint64_t hash_value(float v)
{
    if (v == 0.f) {
        v = 0.f; // -0 and +0 compare equal, so they must hash equal
    }
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    return hash_mix(bits);
}
