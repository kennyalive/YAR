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

// Non-owning string. The characters must outlive the view.
// The characters need not be zero-terminated.
struct String_View
{
    const char* data = nullptr;
    size_t size = 0;
    String_View() = default;
    String_View(const char* s); // s can't be null
    String_View(const char* s, size_t n) : data(s), size(n) {} // s can be null when n is zero
};

bool operator==(String_View a, String_View b);
bool operator!=(String_View a, String_View b);
bool operator<(String_View a, String_View b);
bool equals_ignore_case(String_View a, String_View b);

// Immutable string that owns its characters.
// There is no mutation API: build text elsewhere (string_printf, a local buffer).
// data() is never null and always points to a zero-terminated sequence.
//
// Strings of up to max_small characters (31 by default) live inside the object.
// Longer strings use heap storage. A String object occupies 32 bytes.
struct String
{
    String() { reset_storage(); }
    String(const char* s); // s can't be null
    String(const char* s, size_t n); // s can be null when n is zero
    String(String_View v) : String(v.data, v.size) {}
    String(const String& other);
    String(String&& other) noexcept;
    ~String();
    String& operator=(const String& other);
    String& operator=(String&& other) noexcept;
    operator String_View() const { return {data(), size()}; }

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
    void reset_storage() { storage.small[0] = 0; storage.small[max_small] = char(max_small); }
    uint8_t last_byte() const { return ((const unsigned char*)&storage)[object_size - 1]; }
    bool is_small() const { return last_byte() != heap_tag; }
};
static_assert(sizeof(String) == String::object_size);

String string_printf(const char* format, ...);
String string_concat(String_View a, String_View b);
String string_concat(String_View a, String_View b, String_View c);
String string_concat(String_View a, String_View b, String_View c, String_View d);

// Hashing.
// 
// hash_value(x) returns a 64-bit hash whose bits look uniformly random
// even for structured inputs (round floats, small integers).
// Types add support by overloading hash_value.
// Compound types fold member hashes with hash_combine.
//
// hash_mix is the SplitMix64 finalizer (bijection on 64-bit values)
inline uint64_t hash_mix(uint64_t h)
{
    h ^= h >> 30; h *= 0xbf58476d1ce4e5b9ull;
    h ^= h >> 27; h *= 0x94d049bb133111ebull;
    h ^= h >> 31;
    return h;
}

inline uint64_t hash_value(uint32_t v) { return hash_mix(v); }
inline uint64_t hash_value(int32_t v) { return hash_mix(uint32_t(v)); }
inline uint64_t hash_value(uint64_t v) { return hash_mix(v); }
uint64_t hash_value(float v);

// Boost hash_combine with a 64-bit constant. Inputs must already be well mixed hashes
inline void hash_combine(uint64_t& seed, uint64_t hash)
{
    seed ^= hash + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
}

struct Hasher
{
    template <typename T>
    size_t operator()(const T& v) const { return size_t(hash_value(v)); }
};
