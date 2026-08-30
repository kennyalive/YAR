#pragma once

#include <stddef.h>
#include <stdint.h>

#ifndef MINILIB_SPAN_DEFINED
#define MINILIB_SPAN_DEFINED
#include <initializer_list> // light std include
template <typename T>
struct Span
{
    T* data = nullptr;
    size_t size = 0;

    Span() = default;
    Span& operator=(std::initializer_list<T>) = delete;
    Span(T* data, size_t size) : data(data), size(size) {}
    template <size_t N>
    Span(T(&array)[N]) : data(array), size(N) {}
    template <typename U>
    Span(const Span<U>& other) : data(other.data), size(other.size) {}

    template <typename Container>
    // Constrain to real containers, otherwise an unconstrained Container& matches any lvalue.
    requires requires(Container& container) { container.data(); container.size(); }
    Span(Container& container)
    : data(container.data()), size(container.size()) {}

    // Allows calls like foo({1, 2, 3}) without declaring a separate array.
    // The initializer list storage expires at the end of the full expression.
    // Do not retain the resulting Span.
    Span(std::initializer_list<T> values)
    // initializer_list elements are const, so mutable spans are rejected
    requires requires(const T* data) { static_cast<T*>(data); }
    : data(values.begin()), size(values.size()) {}

    T& operator[](size_t index) const { return data[index]; }
    T* begin() const { return data; }
    T* end() const { return data + size; }
    bool empty() const { return size == 0; }
};
#endif // MINILIB_SPAN_DEFINED

// Non-owning reference to a callable. The callable must outlive this object.
// Mutable lambdas are intentionally unsupported. Capture mutable state by reference instead.
#ifndef MINILIB_FUNCTION_REF_DEFINED
#define MINILIB_FUNCTION_REF_DEFINED
template <typename> struct Function_Ref;
template <typename R, typename... Args>
struct Function_Ref<R(Args...)>
{
    const void* object;
    R(*invoke)(const void*, Args...);
    R operator()(Args... args) const { return invoke(object, static_cast<Args&&>(args)...); }

    template <typename F>
    Function_Ref(const F& f)
        : object(&f)
        , invoke([](const void* object, Args... args) -> R {
            return (*static_cast<const F*>(object))(static_cast<Args&&>(args)...);
        })
    {}
};
#endif // MINILIB_FUNCTION_REF_DEFINED

// Immutable string that owns its characters. data() is never null and always points at
// zero-terminated characters. There is no mutation API: build text elsewhere (string_printf,
// a local buffer) and make a String of it.
//
// Strings of up to max_small characters live inside the object (small string optimization),
// longer ones in a heap allocation. Moving a small string copies it, so pointers obtained
// from data() are valid until the String is moved, reassigned or destroyed.
#ifndef MINILIB_STRING_DEFINED
#define MINILIB_STRING_DEFINED
struct String
{
    static constexpr uint32_t object_size = 32;            // two per cache line
    static constexpr uint32_t max_small = object_size - 1; // characters that fit inside the object

    String() { storage.small[0] = 0; storage.small[max_small] = char(max_small); }
    String(const char* s);              // copies s, which must not be null
    String(const char* s, uint32_t n);  // copies n bytes of s; s may be null when n is 0
    String(const String& other);
    String(String&& other) noexcept;    // takes over a heap string, copies a small one
    String& operator=(String other) noexcept { swap(other); return *this; }
    ~String();

    const char* data() const { return is_small() ? storage.small : storage.heap.chars; }
    uint32_t size() const { return is_small() ? max_small - last_byte() : storage.heap.count; }
    bool empty() const { return size() == 0; }
    void swap(String& other) noexcept;

private:
    // The last byte of the object is shared by both layouts. A small string keeps
    // max_small - size there, which is 0, the terminator, when the string is full.
    // A heap string keeps heap_tag there, which is never a valid spare count.
    static constexpr uint8_t heap_tag = 0x80;
    struct Heap {
        const char* chars;
        uint32_t count;
        uint8_t unused[object_size - sizeof(const char*) - sizeof(uint32_t) - 1];
        uint8_t tag;
    };
    union Storage {
        char small[object_size];
        Heap heap;
    } storage;
    static_assert(sizeof(Storage) == object_size); // no trailing padding, so tag is the last byte

    // Reads the shared byte through the object representation, whichever layout is active.
    uint8_t last_byte() const { return reinterpret_cast<const unsigned char*>(&storage)[max_small]; }
    bool is_small() const { return last_byte() != heap_tag; }
};
static_assert(sizeof(String) == String::object_size);

bool operator==(const String& a, const String& b);
bool operator==(const String& a, const char* b);
bool operator<(const String& a, const String& b);

String string_printf(const char* format, ...);
#endif // MINILIB_STRING_DEFINED
