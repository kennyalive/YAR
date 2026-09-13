#pragma once

#include "minilib.h"

// Path helpers. They look only at the characters and never touch the filesystem.
// Both '/' and '\\' are directory separators on every platform.
//
// Extension rule: the extension is the text after the last dot of the last path
// component, not including the dot. Dots at the start of a name do not count,
// so ".hidden", "." and ".." have no extension, while ".hidden.txt" has "txt".
// Examples: "a.tar.gz" -> "gz", "dir.d/scene" -> none, "file." -> none.
//
// Functions returning String_View return a view into the input, so the result
// is valid exactly as long as the input characters are.

// "pbrt" for "dir/scene.pbrt".
// Empty if there is no extension
String_View path_extension(String_View path);

// "scene.pbrt" for "dir/scene.pbrt".
// Empty if the path ends with a separator
String_View path_filename(String_View path);

// "scene" for "dir/scene.pbrt": the last component without its extension
String_View path_stem(String_View path);

// "dir/" for "dir/scene.pbrt", keeping the separator.
// Empty if there is no separator
String_View path_strip_filename(String_View path);

// "dir/scene.txt" for ("dir/scene.pbrt", "txt").
// An empty extension removes the existing one
String path_replace_extension(String_View path, String_View extension);

// "dir/file" for ("dir", "file"). A '/' is inserted unless a is empty
// or already ends with a separator
String path_join(String_View a, String_View b);
