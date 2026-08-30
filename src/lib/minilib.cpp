#include "minilib.h"

#include <stddef.h>
#include <string.h>

namespace
{
char* copy_string(const char* string, uint32_t size)
{
    char* copy = new char[static_cast<size_t>(size) + 1];
    memcpy(copy, string, size);
    copy[size] = 0;
    return copy;
}
}

String::String()
{
    set_empty();
}

String::String(const char* string)
    : String(string, static_cast<uint32_t>(strlen(string)))
{}

String::String(const char* string, uint32_t size)
{
    if (size <= SMALL_CAPACITY) {
        if (size)
            memcpy(storage.small, string, size);
        storage.small[size] = 0;
        storage.small[SMALL_CAPACITY] = static_cast<char>(SMALL_CAPACITY - size);
    }
    else {
        storage.heap.data = copy_string(string, size);
        storage.heap.size = size;
        storage.heap.tag = HEAP_TAG;
    }
}

String::String(const String& other)
{
    if (other.is_small()) {
        memcpy(&storage, &other.storage, sizeof(storage));
    }
    else {
        storage.heap.data = copy_string(other.storage.heap.data, other.storage.heap.size);
        storage.heap.size = other.storage.heap.size;
        storage.heap.tag = HEAP_TAG;
    }
}

String::String(String&& other) noexcept
{
    memcpy(&storage, &other.storage, sizeof(storage));
    other.set_empty();
}

String::~String()
{
    if (!is_small())
        delete[] storage.heap.data;
}

void String::swap(String& other) noexcept
{
    unsigned char temporary[sizeof(storage)];
    memcpy(temporary, &storage, sizeof(storage));
    memcpy(&storage, &other.storage, sizeof(storage));
    memcpy(&other.storage, temporary, sizeof(storage));
}

bool operator==(const String& a, const String& b)
{
    return a.data() == b.data() || (a.size() == b.size() && memcmp(a.data(), b.data(), a.size()) == 0);
}

bool operator==(const String& a, const char* b)
{
    size_t b_size = strlen(b);
    return a.size() == b_size && memcmp(a.data(), b, a.size()) == 0;
}

bool operator<(const String& a, const String& b)
{
    uint32_t size = a.size() < b.size() ? a.size() : b.size();
    int comparison = memcmp(a.data(), b.data(), size);
    return comparison < 0 || (comparison == 0 && a.size() < b.size());
}

void String::set_empty()
{
    storage.small[0] = 0;
    storage.small[SMALL_CAPACITY] = static_cast<char>(SMALL_CAPACITY);
}
