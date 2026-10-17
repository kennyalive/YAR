#include "minilib.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include <cstddef> // std::max_align_t

#ifdef _WIN32
#include <malloc.h>
#endif

//
// Array storage
//
void* minilib_internal::allocate_array_storage(size_t count, size_t element_size, size_t alignment)
{
    // For invalid large inputs, multiplication can wrap around to a small size
    // that can be allocated successfully, hiding the bug
    if (count > SIZE_MAX / element_size) {
        abort();
    }

    const size_t size = count * element_size;
    void* data;
    if (alignment <= alignof(std::max_align_t)) {
        data = malloc(size);
    }
    else {
#ifdef _WIN32
        data = _aligned_malloc(size, alignment);
#else
        if (posix_memalign(&data, alignment, size) != 0) {
            data = nullptr;
        }
#endif
    }
    if (!data) {
        abort();
    }
    return data;
}

void minilib_internal::free_array_storage(void* data, size_t alignment)
{
#ifdef _WIN32
    if (alignment > alignof(std::max_align_t)) {
        _aligned_free(data);
        return;
    }
#endif
    (void)alignment;
    free(data);
}

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

static char ascii_to_lower(char c)
{
    return (c >= 'A' && c <= 'Z') ? char(c + ('a' - 'A')) : c;
}

// Case is ignored for ascii letters only, other bytes must match exactly
bool equals_ignore_case(String_View a, String_View b)
{
    if (a.size != b.size) {
        return false;
    }
    for (size_t i = 0; i < a.size; i++) {
        if (ascii_to_lower(a.data[i]) != ascii_to_lower(b.data[i])) {
            return false;
        }
    }
    return true;
}

bool parse_int(String_View text, int32_t& result)
{
    size_t i = 0;
    const bool negative = text.size && text.data[0] == '-';
    if (negative) {
        i++;
    }
    if (i == text.size) {
        return false;
    }
    int64_t value = 0;
    for (; i < text.size; i++) {
        const int32_t digit = text.data[i] - '0';
        if (digit < 0 || digit > 9) {
            return false;
        }
        value = value * 10 + digit;
        // Bound the accumulator on long input while allowing INT32_MIN's magnitude
        if (value > int64_t(INT32_MAX) + 1) {
            return false;
        }
    }
    if (negative) {
        value = -value;
    }
    if (value > INT32_MAX) {
        return false;
    }
    result = int32_t(value);
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
        const char* p = heap_copy(s, n);
        storage.heap.init(p, n);
    }
}

String::String(const String& other)
{
    if (other.is_small()) {
        storage = other.storage;
    }
    else {
        const char* p = heap_copy(other.storage.heap.chars, other.storage.heap.count);
        storage.heap.init(p, other.storage.heap.count);
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
        const char* p = heap_copy(other.storage.heap.chars, other.storage.heap.count);
        storage.heap.init(p, other.storage.heap.count);
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
    str.storage.heap.init(alloced_buffer, n);
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
        result.storage.heap.init(p, total);
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

String string_to_lower(String_View s)
{
    String str(s);
    char* p = (char*)str.data(); // str owns these characters, so folding them in place is fine
    for (size_t i = 0; i < s.size; i++) {
        p[i] = ascii_to_lower(p[i]);
    }
    return str;
}

//
// File IO
//
bool File::close()
{
    if (!file) {
        return true;
    }
    bool closed = (fclose(file) == 0);
    file = nullptr;
    return closed;
}

bool File::read(void* destination, size_t size)
{
    ASSERT(file);
    ASSERT(destination || !size);
    return !size || fread(destination, 1, size, file) == size;
}

bool File::write(const void* source, size_t size)
{
    ASSERT(file);
    ASSERT(source || !size);
    return !size || fwrite(source, 1, size, file) == size;
}

Scoped_File::~Scoped_File()
{
    close();
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
