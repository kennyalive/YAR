#include "platform.h"
#include <atomic>
#include <chrono>

bool fs_load(String_View path, Array<uint8_t>& bytes)
{
    Scoped_File file = fs_open(path, "rb");
    if (!file) {
        return false;
    }
    size_t size{};
    if (!fs_file_size(file, size)) {
        return false;
    }
    Array<uint8_t> content;
    content.resize_uninitialized(size);
    if (!file.read(content.data(), size)) {
        return false;
    }
    bytes = rvalue(content);
    return true;
}

bool fs_save(String_View path, const void* data, size_t size)
{
    ASSERT(data || !size);
    Scoped_File file = fs_open(path, "wb");
    if (!file) {
        return false;
    }
    if (!file.write(data, size)) {
        return false;
    }
    return file.close();
}

bool fs_load_text(String_View path, String& text)
{
    Array<uint8_t> content;
    if (!fs_load(path, content)) {
        return false;
    }
    text = String(reinterpret_cast<const char*>(content.data()), content.size());
    return true;
}

bool fs_save_text(String_View path, String_View text)
{
    return fs_save(path, text.data, text.size);
}

Timestamp::Timestamp()
    : nanoseconds(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count())
{}

int64_t elapsed_nanoseconds(Timestamp start)
{
    return Timestamp().nanoseconds - start.nanoseconds;
}

int64_t elapsed_milliseconds(Timestamp start)
{
    return elapsed_nanoseconds(start) / 1'000'000;
}

float elapsed_seconds(Timestamp start)
{
    return float(double(elapsed_nanoseconds(start)) / 1e9);
}

//
// Threads
//
Thread::Thread(Thread&& other) noexcept
{
    handle = other.handle;
    other.handle = 0;
}

Thread& Thread::operator=(Thread&& other) noexcept
{
    if (this != &other) {
        ASSERT(!handle); // destination must be empty
        handle = other.handle;
        other.handle = 0;
    }
    return *this;
}

static void run_thread_body(void* context)
{
    (*static_cast<Function_Ref<void()>*>(context))();
}

void run_on_threads(Function_Ref<void()> function, uint32_t thread_count)
{
    if (thread_count == 0) {
        thread_count = logical_processor_count();
    }
    ASSERT(thread_count >= 1);
    Array<Thread> threads;
    threads.reserve(thread_count - 1);
    for (uint32_t i = 1; i < thread_count; i++) {
        threads.add(thread_start(run_thread_body, &function));
    }
    function();
    for (Thread& thread : threads) {
        thread_join(thread);
    }
}

//
// Atomics
//
namespace minilib_internal {
template <typename T>
static std::atomic_ref<T> atomic_value(T& value)
{
    static_assert(alignof(Atomic<T>) >= std::atomic_ref<T>::required_alignment);
    static_assert(std::atomic_ref<T>::is_always_lock_free);
    return std::atomic_ref<T>(value);
}

template <typename T>
T Atomic<T>::load() const
{
    return atomic_value(value).load();
}

template <typename T>
void Atomic<T>::store(T desired)
{
    atomic_value(value).store(desired);
}

template <typename T>
T Atomic<T>::fetch_add(T amount)
{
    return atomic_value(value).fetch_add(amount);
}

template <typename T>
T Atomic<T>::fetch_sub(T amount)
{
    return atomic_value(value).fetch_sub(amount);
}

template <typename T>
T Atomic<T>::exchange(T desired)
{
    return atomic_value(value).exchange(desired);
}

template <typename T>
bool Atomic<T>::compare_exchange(T& expected, T desired)
{
    return atomic_value(value).compare_exchange_strong(expected, desired);
}

template struct Atomic<bool>;
template struct Atomic<int32_t>;
template struct Atomic<uint32_t>;
template struct Atomic<int64_t>;
template struct Atomic<uint64_t>;
} // namespace minilib_internal
