#pragma once

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
