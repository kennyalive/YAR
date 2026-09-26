#include "platform.h"
#include "path.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <limits.h>
#include <malloc.h>
#include <string.h>
#include <sys/stat.h>
#include <wchar.h>
#include <xmmintrin.h>

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
    return _wfopen(name, wide_mode);
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

void* allocate_aligned_memory(size_t size, size_t alignment)
{
    if (size == 0 || alignment == 0 || (alignment & (alignment - 1))) {
        return nullptr;
    }
    return _aligned_malloc(size, alignment);
}

void free_aligned_memory(void* memory) { _aligned_free(memory); }

int logical_processor_count()
{
    DWORD count = GetActiveProcessorCount(ALL_PROCESSOR_GROUPS);
    return count ? int(count) : 1;
}

void enable_invalid_fp_exception()
{
    _MM_SET_EXCEPTION_STATE(0);
    _MM_SET_EXCEPTION_MASK(_MM_MASK_MASK & ~_MM_MASK_INVALID);
}
