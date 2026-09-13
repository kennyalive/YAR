#include "path.h"

static bool is_separator(char c)
{
    return c == '/' || c == '\\';
}

static size_t filename_start_index(String_View path)
{
    for (size_t i = path.size; i > 0; i--) {
        if (is_separator(path.data[i - 1])) {
            return i;
        }
    }
    return 0;
}

// Index of the dot that starts the extension, or path.size if there is no extension.
// This is the single place that defines what counts as an extension
static size_t extension_dot_index(String_View path)
{
    size_t start = filename_start_index(path);
    while (start < path.size && path.data[start] == '.') {
        start++; // leading dots never start an extension
    }
    for (size_t i = path.size; i > start; i--) {
        if (path.data[i - 1] == '.') {
            return i - 1;
        }
    }
    return path.size; // no dot, or only leading dots
}

String_View path_extension(String_View path)
{
    size_t dot = extension_dot_index(path);
    if (dot == path.size) {
        return {};
    }
    return {path.data + dot + 1, path.size - dot - 1};
}

String_View path_filename(String_View path)
{
    size_t start = filename_start_index(path);
    return {path.data + start, path.size - start};
}

String_View path_stem(String_View path)
{
    size_t start = filename_start_index(path);
    size_t dot = extension_dot_index(path);
    return {path.data + start, dot - start};
}

String_View path_strip_filename(String_View path)
{
    return {path.data, filename_start_index(path)};
}

String path_replace_extension(String_View path, String_View extension)
{
    String_View base{path.data, extension_dot_index(path)};
    if (extension.size == 0) {
        return base;
    }
    return string_concat(base, ".", extension);
}

String path_join(String_View a, String_View b)
{
    if (a.size == 0) {
        return b;
    }
    if (is_separator(a.data[a.size - 1])) {
        return string_concat(a, b);
    }
    return string_concat(a, "/", b);
}
