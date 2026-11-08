#include "minilib_platform.h"
#include <atomic>
#include <chrono>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <limits.h>
#include <process.h>
#include <share.h>
#include <sys/stat.h>
#include <wchar.h>
#include <xmmintrin.h>
#endif

#if MINILIB_USE_NAMESPACE
namespace mini {
#endif

// ============================================================================
// Shared implementation
// ============================================================================

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

//
// Timing
//
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
namespace mini_internal {
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
} // namespace mini_internal

#if defined(_WIN32)
// ============================================================================
// Windows implementation
// ============================================================================

struct Wide_String
{
    wchar_t* data = nullptr;
    operator const wchar_t* () const { return data; }
    Wide_String(String_View text)
    {
        ASSERT(text.data || !text.size);
        if (!text.size || text.size > INT_MAX) {
            return;
        }
        int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data, int(text.size), nullptr, 0);
        if (!n) {
            return;
        }
        data = new wchar_t[size_t(n) + 1];
        MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data, int(text.size), data, n);
        data[n] = 0;
    }
    Wide_String(const wchar_t* directory, const wchar_t* name)
    {
        size_t n = wcslen(directory);
        size_t m = wcslen(name);
        data = new wchar_t[n + m + 2];
        memcpy(data, directory, n * sizeof(wchar_t));
        if (n && directory[n - 1] != L'\\' && directory[n - 1] != L'/') {
            data[n++] = L'\\';
        }
        memcpy(data + n, name, (m + 1) * sizeof(wchar_t));
    }
    ~Wide_String() { delete[] data; }
    Wide_String(const Wide_String&) = delete;
    Wide_String& operator=(const Wide_String&) = delete;
};

static bool convert_filename_to_utf8(const wchar_t* filename, String& result)
{
    // UTF-8 needs at most 3 bytes per UTF-16 code unit
    char buffer[MAX_PATH * 3];

    int n = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, filename, -1, buffer, sizeof(buffer), nullptr, nullptr);
    if (!n) {
        return false;
    }
    result = String(buffer, size_t(n - 1));
    return true;
}

static bool missing(DWORD code)
{
    return code == ERROR_FILE_NOT_FOUND || code == ERROR_PATH_NOT_FOUND;
}

static bool scan_directory(const wchar_t* directory, Function_Ref<bool(const WIN32_FIND_DATAW&)> visit, bool* scan_error = nullptr)
{
    if (scan_error) {
        *scan_error = false;
    }
    struct Directory_Search {
        HANDLE handle;
        ~Directory_Search() {
            if (handle != INVALID_HANDLE_VALUE) {
                FindClose(handle);
            }
        }
    };
    Wide_String pattern(directory, L"*");
    WIN32_FIND_DATAW entry;
    Directory_Search search{FindFirstFileW(pattern, &entry)};

    if (search.handle == INVALID_HANDLE_VALUE) {
        if (scan_error) {
            *scan_error = GetLastError() != ERROR_FILE_NOT_FOUND;
        }
        return true;
    }
    do {
        if (wcscmp(entry.cFileName, L".") == 0 ||
            wcscmp(entry.cFileName, L"..") == 0) {
            continue;
        }
        if (!visit(entry)) {
            return false;
        }
    } while (FindNextFileW(search.handle, &entry));
    if (scan_error) {
        *scan_error = GetLastError() != ERROR_NO_MORE_FILES;
    }
    return true;
}

static bool remove_directory_contents(const wchar_t* directory)
{
    return scan_directory(directory, [&](const WIN32_FIND_DATAW& entry) {
        Wide_String child(directory, entry.cFileName);
        DWORD attrs = entry.dwFileAttributes;
        if (!(attrs & FILE_ATTRIBUTE_DIRECTORY)) {
            return DeleteFileW(child) != 0;
        }
        if (!(attrs & FILE_ATTRIBUTE_REPARSE_POINT) && !remove_directory_contents(child)) {
            return false;
        }
        return RemoveDirectoryW(child) != 0;
    });
}

FILE* fs_open(String_View path, const char* mode)
{
    ASSERT(mode && mode[0]);
    Wide_String name(path);
    if (!name) {
        return nullptr;
    }
    Wide_String wide_mode(mode);
    if (!wide_mode) {
        return nullptr;
    }
    return _wfsopen(name, wide_mode, _SH_DENYNO);
}

bool fs_file_size(FILE* file, size_t& size)
{
    struct _stat64 st;
    if (_fstat64(_fileno(file), &st) != 0 ||
        (st.st_mode & _S_IFMT) != _S_IFREG) {
        return false;
    }
    size = size_t(st.st_size);
    return true;
}

bool fs_create_directory(String_View path)
{
    Wide_String name(path);
    if (!name) {
        return false;
    }
    DWORD attrs = GetFileAttributesW(name);
    if (attrs != INVALID_FILE_ATTRIBUTES) {
        return (attrs & FILE_ATTRIBUTE_DIRECTORY) != 0;
    }
    String_View parent = path_parent(path);
    if (parent.size && parent != path && !fs_create_directory(parent)) {
        return false;
    }
    return CreateDirectoryW(name, nullptr) != 0;
}

bool fs_remove_tree(String_View path)
{
    Wide_String name(path);
    if (!name) {
        return false;
    }
    DWORD attrs = GetFileAttributesW(name);
    if (attrs == INVALID_FILE_ATTRIBUTES) {
        return missing(GetLastError());
    }
    if (!(attrs & FILE_ATTRIBUTE_DIRECTORY)) {
        return DeleteFileW(name) != 0;
    }
    if (!(attrs & FILE_ATTRIBUTE_REPARSE_POINT) && !remove_directory_contents(name)) {
        return false;
    }
    return RemoveDirectoryW(name) != 0;
}

bool fs_rename_file(String_View from, String_View to)
{
    Wide_String source(from);
    Wide_String destination(to);
    if (!source || !destination) {
        return false;
    }
    DWORD attrs = GetFileAttributesW(source);
    if (attrs == INVALID_FILE_ATTRIBUTES || (attrs & FILE_ATTRIBUTE_DIRECTORY)) {
        return false;
    }
    return MoveFileExW(source, destination, 0) != 0;
}

bool fs_remove_file(String_View path)
{
    Wide_String name(path);
    if (!name) {
        return false;
    }
    return DeleteFileW(name) || missing(GetLastError());
}

void fs_for_each_file(String_View directory, Function_Ref<void(String_View)> visit)
{
    Wide_String name(directory);
    if (!name) {
        return;
    }
    scan_directory(name, [&](const WIN32_FIND_DATAW& entry) {
        if (!(entry.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
            String filename;
            if (!convert_filename_to_utf8(entry.cFileName, filename)) {
                return true;
            }
            String path = path_join(directory, filename);
            visit(path);
        }
        return true;
    });
}

bool fs_exists(String_View path)
{
    Wide_String name(path);
    if (!name) {
        return false;
    }
    return GetFileAttributesW(name) != INVALID_FILE_ATTRIBUTES;
}

bool fs_is_directory_empty(String_View directory)
{
    Wide_String name(directory);
    if (!name) {
        return false;
    }
    bool scan_error;
    bool empty = scan_directory(name, [](const WIN32_FIND_DATAW&) {
        return false;
    }, &scan_error);
    return empty && !scan_error;
}

void enable_invalid_fp_exception()
{
    _MM_SET_EXCEPTION_STATE(0);
    _MM_SET_EXCEPTION_MASK(_MM_MASK_MASK & ~_MM_MASK_INVALID);
}

//
// Threads
//
struct Thread_Start
{
    void (*function)(void*);
    void* context;
};

static unsigned thread_entry(void* data)
{
    Thread_Start* storage = static_cast<Thread_Start*>(data);
    const Thread_Start start = *storage;
    delete storage;
    start.function(start.context);
    return 0;
}

Thread thread_start(void (*function)(void*), void* data)
{
    static_assert(sizeof(Thread::handle) >= sizeof(HANDLE));
    ASSERT(function);

    Thread_Start* start = new Thread_Start{function, data};

    Thread thread;
    thread.handle = _beginthreadex(nullptr, 0, thread_entry, start, 0, nullptr);
    if (!thread.handle) {
        abort();
    }
    return thread;
}

void thread_join(Thread& thread)
{
    if (!thread.handle) {
        return;
    }
    HANDLE native_handle = reinterpret_cast<HANDLE>(thread.handle);
    ASSERT(::GetThreadId(native_handle) != ::GetCurrentThreadId());

    if (WaitForSingleObject(native_handle, INFINITE) != WAIT_OBJECT_0) {
        abort();
    }
    if (!CloseHandle(native_handle)) {
        abort();
    }
    thread.handle = 0;
}

uint32_t logical_processor_count()
{
    DWORD count = GetActiveProcessorCount(ALL_PROCESSOR_GROUPS);
    return count ? uint32_t(count) : 1;
}

//
// Locks
//
Mutex::Mutex()
{
    static_assert(sizeof(storage) >= sizeof(SRWLOCK));
    static_assert(alignof(Mutex) >= alignof(SRWLOCK));
    InitializeSRWLock(reinterpret_cast<SRWLOCK*>(storage));
}

Mutex::~Mutex()
{
    // SRW locks need no cleanup
}

void Mutex::lock()
{
    AcquireSRWLockExclusive(reinterpret_cast<SRWLOCK*>(storage));
}

void Mutex::unlock()
{
    ReleaseSRWLockExclusive(reinterpret_cast<SRWLOCK*>(storage));
}
#else
#error platform backend is not implemented
#endif

#if MINILIB_USE_NAMESPACE
} // namespace mini
#endif
