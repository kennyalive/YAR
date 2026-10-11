#pragma once

constexpr int MINILIB_VERSION = 0;

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <initializer_list>

#ifndef ENABLE_ASSERT
#define ENABLE_ASSERT 1
#endif

#if ENABLE_ASSERT
#define ASSERT(expression) if (expression) {} else __debugbreak()
#else
#define ASSERT(expression)
#endif

//
// Basic type and value utilities
//
template <typename T>
T&& rvalue(T& value)
{
    return static_cast<T&&>(value);
}

template <typename T>
constexpr bool is_bitwise_copyable()
{
    // supported by MSVC, GCC and Clang (including Apple Clang)
    return __is_trivially_copyable(T);
}

template <typename T>
constexpr bool has_trivial_destructor()
{
#if defined(__GNUC__) && !defined(__clang__)
    return __has_trivial_destructor(T);
#else
    // MSVC and Clang (including Apple Clang)
    return __is_trivially_destructible(T);
#endif
}

template <typename T> constexpr bool is_raw_pointer = false;
template <typename T> constexpr bool is_raw_pointer<T*> = true;

// Whether zeroing the bytes is equivalent to T()
// Specialize for custom types that satisfy this condition
template <typename T>
constexpr bool can_bitwise_zero_initialize()
{
    if constexpr (is_raw_pointer<T>) {
        return true;
    }
    if constexpr (__is_trivially_constructible(T)) {
        // Trivial assignment from int selects numeric types, excluding pointers and enums
        return __is_trivially_assignable(T&, int);
    }
    return false;
}

template <typename T>
void swap_values(T& a, T& b)
{
    T temp = rvalue(a);
    a = rvalue(b);
    b = rvalue(temp);
}

//
// Non-owning view of contiguous elements
//
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
    // Prevent this overload from being chosen when copying a Span
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

// Custom placement new avoids including <new> in this header
struct Placement_New {};
inline void* operator new(size_t, Placement_New, void* data) noexcept { return data; }
inline void operator delete(void*, Placement_New, void*) noexcept {}

void* allocate_array_storage(size_t count, size_t element_size, size_t alignment);
void free_array_storage(void* data, size_t alignment);

//
// Dynamic array that owns its elements
//
template <typename T>
struct Array
{
    Array() = default;
    explicit Array(size_t size)
    {
        resize(size);
    }
    Array(size_t size, const T& value)
    {
        auto construct = [&](T* destination, size_t n) {
            for (size_t i = 0; i < n; i++) {
                ::new (Placement_New{}, destination + i) T(value);
            }
        };
        append(size, construct);
    }
    Array(std::initializer_list<T> values)
    {
        add_all({values.begin(), values.size()});
    }
    Array(const Array& other)
    {
        add_all(other);
    }
    Array(Array&& other) noexcept
    {
        swap(other);
    }
    ~Array()
    {
        clear();
        free_storage(elements);
    }

    Array& operator=(const Array& other)
    {
        if (this != &other) {
            Array copy(other);
            swap(copy);
        }
        return *this;
    }
    Array& operator=(Array&& other) noexcept
    {
        Array moved(rvalue(other));
        swap(moved);
        return *this;
    }

    T* data() { return elements; }
    const T* data() const { return elements; }
    size_t size() const { return count; }
    bool empty() const { return count == 0; }

    T* begin() { return elements; }
    const T* begin() const { return elements; }
    T* end() { return elements + count; }
    const T* end() const { return elements + count; }

    Span<T> span() { return {elements, count}; }
    Span<const T> span() const { return {elements, count}; }

    T& operator[](size_t index)
    {
        ASSERT(index < count);
        return elements[index];
    }
    const T& operator[](size_t index) const
    {
        ASSERT(index < count);
        return elements[index];
    }

    T& last()
    {
        ASSERT(count);
        return elements[count - 1];
    }
    const T& last() const
    {
        ASSERT(count);
        return elements[count - 1];
    }

    T& add()
    {
        append(1, construct_elements);
        return last();
    }
    void add(const T& value)
    {
        auto construct = [&](T* destination, size_t) {
            ::new (Placement_New{}, destination) T(value);
        };
        append(1, construct);
    }
    void add(T&& value)
    {
        auto construct = [&](T* destination, size_t) {
            ::new (Placement_New{}, destination) T(rvalue(value));
        };
        append(1, construct);
    }
    void add_all(Span<const T> values)
    {
        auto construct = [&](T* destination, size_t n) {
            for (size_t i = 0; i < n; i++) {
                ::new (Placement_New{}, destination + i) T(values[i]);
            }
        };
        append(values.size, construct);
    }

    void reserve(size_t capacity)
    {
        if (capacity <= allocated) {
            return;
        }
        T* new_elements = allocate_storage(capacity);
        move_elements(new_elements, elements, count);
        destroy_elements(elements, count);
        free_storage(elements);
        elements = new_elements;
        allocated = capacity;
    }

    void resize(size_t size)
    {
        if (size < count) {
            destroy_elements(elements + size, count - size);
            count = size;
            return;
        }
        append(size - count, construct_elements);
    }

    void clear()
    {
        destroy_elements(elements, count);
        count = 0;
    }

    void swap(Array& other) noexcept
    {
        swap_values(elements, other.elements);
        swap_values(count, other.count);
        swap_values(allocated, other.allocated);
    }

    // Leaves newly added slots uninitialized. Construct them before any
    // operation that reads, moves or destroys those elements
    void resize_uninitialized(size_t size)
    {
        if (size < count) {
            destroy_elements(elements + size, count - size);
        }
        else {
            reserve(size);
        }
        count = size;
    }

    // Constructs an element in an uninitialized slot without changing size
    template <typename... Args>
    void construct_uninitialized_at(size_t index, Args&&... args)
    {
        ASSERT(index < count);
        ::new (Placement_New{}, elements + index) T(static_cast<Args&&>(args)...);
    }

private:
    T* elements = nullptr;
    size_t count = 0;
    size_t allocated = 0;

    static T* allocate_storage(size_t capacity)
    {
        return static_cast<T*>(allocate_array_storage(capacity, sizeof(T), alignof(T)));
    }
    static void free_storage(T* data)
    {
        free_array_storage(data, alignof(T));
    }
    static void construct_elements(T* data, size_t size)
    {
        if constexpr (can_bitwise_zero_initialize<T>()) {
            if (size) {
                memset(data, 0, size * sizeof(T));
            }
        } else {
            for (size_t i = 0; i < size; i++) {
                ::new (Placement_New{}, data + i) T();
            }
        }
    }
    static void destroy_elements(T* data, size_t size)
    {
        if constexpr (!has_trivial_destructor<T>()) {
            for (size_t i = size; i > 0; i--) {
                data[i - 1].~T();
            }
        }
    }
    static void move_elements(T* destination, T* source, size_t size)
    {
        if constexpr (is_bitwise_copyable<T>()) {
            if (size) {
                memcpy(destination, source, size * sizeof(T));
            }
        } else {
            for (size_t i = 0; i < size; i++) {
                ::new (Placement_New{}, destination + i) T(rvalue(source[i]));
            }
        }
    }
    template <typename Construct>
    void append(size_t size, Construct construct)
    {
        const size_t max_count = SIZE_MAX / sizeof(T);
        if (size > max_count - count) {
            abort();
        }
        const size_t new_count = count + size;
        if (new_count <= allocated) {
            construct(elements + count, size);
            count = new_count;
            return;
        }
        size_t capacity = allocated * 2;
        if (capacity < new_count) {
            capacity = new_count;
        }
        T* new_elements = allocate_storage(capacity);
        // Construct additions first to support add(a[0]) and add_all(a.span())
        construct(new_elements + count, size);
        move_elements(new_elements, elements, count);
        destroy_elements(elements, count);
        free_storage(elements);
        elements = new_elements;
        count = new_count;
        allocated = capacity;
    }
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
        void init(const char* chars, size_t count)
        {
            this->chars = chars;
            this->count = count;
            tag = heap_tag;
        }
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
String string_to_lower(String_View s); // ASCII letters only, other bytes unchanged

// FILE* wrapper with convenience helpers.
// Does NOT close file automatically
struct File
{
    FILE* file = nullptr;

    File(FILE* file = nullptr) : file(file) {}
    operator FILE* () const { return file; }

    bool close();
    bool read(void* destination, size_t size);
    bool write(const void* source, size_t size);
};

// Closes file automatically when it exits the scope
struct Scoped_File : File
{
    Scoped_File(FILE* file = nullptr) : File(file) {}
    ~Scoped_File();
    Scoped_File(const Scoped_File&) = delete;
    Scoped_File& operator=(const Scoped_File&) = delete;
};

//
// Optional value
//
template <typename T>
struct Optional
{
    T value{};
    bool has_value = false;

    Optional() = default;
    Optional(const T& value) : value(value), has_value(true) {}
    Optional(T&& value) : value(rvalue(value)), has_value(true) {}

    explicit operator bool() const { return has_value; }
};

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

//
// Misc utilities
//
template <typename T>
T min(T a, T b)
{
    return b < a ? b : a;
}

template <typename T>
T max(T a, T b)
{
    return a < b ? b : a;
}

template <typename T>
T clamp(T value, T low, T high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

template <typename To, typename From>
To bit_cast(const From& value)
{
    // supported by MSVC, GCC and Clang (including Apple Clang)
    return __builtin_bit_cast(To, value);
}
