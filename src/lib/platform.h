#pragma once

#include "minilib.h"
#include <stdio.h>

//
// File IO
//
// The paths are UTF-8. File loads leave the output unchanged on failure.
// Text files are read and written as-is (newlines and BOMs are preserved).
//

// Like fopen, but accepts UTF-8 paths
FILE* fs_open(String_View path, const char* mode);

bool fs_file_size(FILE* file, size_t& size);

// Binary files
bool fs_load(String_View path, Array<uint8_t>& bytes);
bool fs_save(String_View path, const void* data, size_t size);

// Text files
bool fs_load_text(String_View path, String& text);
bool fs_save_text(String_View path, String_View text);

// Creates the requested directory and any missing parent directories.
// Succeeds if the directory already exists
bool fs_create_directory(String_View path);

// Removes a directory and all its contents, or a single file.
// A missing path is not a failure
bool fs_remove_tree(String_View path);

// Same volume on Windows, same filesystem mount on Linux.
// Destination must not exist
bool fs_rename_file(String_View from, String_View to);

// A missing path is not a failure
bool fs_remove_file(String_View path);

// Nonrecursive. Skips directories.
void fs_for_each_file(String_View directory, Function_Ref<void(String_View path)> visit);

// Returns false if the answer is no or cannot be determined
bool fs_exists(String_View path);
bool fs_is_directory_empty(String_View directory);

struct Timestamp
{
    Timestamp(); // captures the current time
    int64_t nanoseconds;
};

int64_t elapsed_nanoseconds(Timestamp start);
int64_t elapsed_milliseconds(Timestamp start);
float elapsed_seconds(Timestamp start);

// Alignment must be a power of two.
// Returns null on failure or when size is zero
void* allocate_aligned_memory(size_t size, size_t alignment);
void free_aligned_memory(void* memory);

uint32_t logical_processor_count(); // at least one
void enable_invalid_fp_exception(); // calling thread only

//
// Threads
//
// Every started thread must be joined, even after it finishes.
// Destruction does not join.
//

struct Thread
{
    uintptr_t handle = 0;

    Thread() = default;
    ~Thread() { ASSERT(!handle); }

    Thread(Thread&& other) noexcept;
    Thread& operator=(Thread&& other) noexcept;

    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;
};

Thread thread_start(void (*function)(void*), void* data);
void thread_join(Thread& thread);

// Runs function once on each of thread_count threads, including the caller.
// Joins before returning. A thread_count of zero uses all logical cores,
// including the caller.
void run_on_threads(Function_Ref<void()> function, uint32_t thread_count = 0);

//
// Atomics
//
// Supported scalar types are lock-free.
// All operations are sequentially consistent
//
namespace minilib_internal {
template <typename T>
struct Atomic
{
    static_assert(
        is_same_type<T, bool> ||
        is_same_type<T, int32_t> || is_same_type<T, uint32_t> ||
        is_same_type<T, int64_t> || is_same_type<T, uint64_t>,
        "Atomic supports bool and 32-/64-bit integers"
    );
    constexpr explicit Atomic(T initial = T{}) : value(initial) {}
    Atomic(const Atomic&) = delete;
    Atomic& operator=(const Atomic&) = delete;

    T load() const;
    void store(T desired);

    // Return the previous value
    T fetch_add(T amount);
    T fetch_sub(T amount);
    T exchange(T desired);

    bool compare_exchange(T& expected, T desired);

private:
    mutable T value{};
};

template <> bool Atomic<bool>::fetch_add(bool) = delete;
template <> bool Atomic<bool>::fetch_sub(bool) = delete;
} // namespace minilib_internal

using Atomic_Bool = minilib_internal::Atomic<bool>;
using Atomic_Int32 = minilib_internal::Atomic<int32_t>;
using Atomic_UInt32 = minilib_internal::Atomic<uint32_t>;
using Atomic_Int64 = minilib_internal::Atomic<int64_t>;
using Atomic_UInt64 = minilib_internal::Atomic<uint64_t>;

//
// Locks
//
struct Mutex // non-recursive
{
    Mutex();
    ~Mutex();
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    void lock();
    void unlock();

private:
#ifdef _WIN32
    alignas(void*) unsigned char storage[sizeof(void*)]; // SRWLOCK
#else
    alignas(8) unsigned char storage[64]; // pthread_mutex_t; backend must verify size and alignment
#endif
};

struct [[nodiscard]] Scoped_Lock
{
    explicit Scoped_Lock(Mutex& mutex) : mutex(mutex) { mutex.lock(); }
    ~Scoped_Lock() { mutex.unlock(); }
    Scoped_Lock(const Scoped_Lock&) = delete;
    Scoped_Lock& operator=(const Scoped_Lock&) = delete;

private:
    Mutex& mutex;
};
