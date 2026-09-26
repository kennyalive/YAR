#pragma once

#include "minilib.h"
#include <stdio.h>

//
// The paths are UTF-8.
// File loads leave the output unchanged on failure.
// Text files are read and written as-is (newlines and BOMs are preserved).
//

// Like fopen, but accepts UTF-8 paths
FILE* fs_open(String_View path, const char* mode);

bool fs_file_size(FILE* file, size_t& size);

// Binary files
bool fs_load(String_View path, Byte_Buffer& bytes);
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

int logical_processor_count(); // at least one
void enable_invalid_fp_exception(); // calling thread only
