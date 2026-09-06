#pragma once

constexpr int MINILIB_VERSION = 0;

#include <stddef.h>
#include <stdint.h>
#include <initializer_list>

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

// Non-owning reference to a callable. The callable must outlive this object.
// Mutable lambdas are intentionally unsupported. Capture mutable state by reference instead.
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

// Immutable string that owns its characters.
// There is no mutation API: build text elsewhere (string_printf, a local buffer).
// data() is never null and always points to a zero-terminated sequence.
//
// Strings of up to max_small characters (31 by default) live inside the object.
// Longer strings use heap storage.
// By default, a String object occupies 32 bytes (two per cache line).
struct String
{
    String() { storage.small[0] = 0; storage.small[max_small] = char(max_small); }
    String(const char* s);
    String(const char* s, size_t n);
    String(const String& other);
    String(String&& other) noexcept;
    ~String();
    String& operator=(const String& other);
    String& operator=(String&& other) noexcept;

    const char* data() const { return is_small() ? storage.small : storage.heap.chars; }
    size_t size() const { return is_small() ? max_small - last_byte() : storage.heap.count; }
    bool empty() const { return size() == 0; }
    const char* c_str() const { return data(); }
    const char* begin() const { return data(); }
    const char* end() const { return data() + size(); }

    static constexpr uint32_t object_size = 32;

    // Characters that fit inside the object
    static constexpr uint32_t max_small = object_size - 1;

    // The last byte identifies the storage layout. Small strings store
    // (max_small - size) in this byte. At full capacity this byte is zero
    // and serves as the terminator. Heap strings store heap_tag in this byte.
    static constexpr uint8_t heap_tag = 0x80;

    struct Heap {
        const char* chars;
        size_t count;
        uint8_t unused[object_size - sizeof(chars) - sizeof(count) - 1 /*tag*/];
        uint8_t tag;
    };
    union Storage {
        char small[object_size];
        Heap heap;
    } storage;

private:
    uint8_t last_byte() const { return ((const unsigned char*)&storage)[object_size - 1]; }
    bool is_small() const { return last_byte() != heap_tag; }
};
static_assert(sizeof(String) == String::object_size);

// Non-owning string. The characters must outlive the view.
// The characters need not be zero-terminated.
struct String_View
{
    const char* data = "";
    size_t size = 0;
    String_View() = default;
    String_View(const String& s) : data(s.data()), size(s.size()) {}
    String_View(const char* s);
    String_View(const char* s, size_t n) : data(s), size(n) {} // s may be null when n is zero
};

bool operator==(const String& a, const String& b);
bool operator!=(const String& a, const String& b);
bool operator==(const String& a, const char* b);
bool operator<(const String& a, const String& b);
String string_printf(const char* format, ...);
String string_concat(String_View a, String_View b);
String string_concat(String_View a, String_View b, String_View c);
String string_concat(String_View a, String_View b, String_View c, String_View d);
