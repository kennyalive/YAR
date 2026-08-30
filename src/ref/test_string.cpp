#include "std.h"
#include "lib/common.h"

static bool string_data_is_inline(const String& string)
{
    uintptr_t data = reinterpret_cast<uintptr_t>(string.data());
    uintptr_t object = reinterpret_cast<uintptr_t>(&string);
    return data >= object && data < object + sizeof(string);
}

void test_string()
{
    String empty;
    ASSERT(empty.empty());
    ASSERT(empty.size() == 0);
    ASSERT(empty.data()[0] == 0);

    String first = "stored string";
    String second = "stored string";
    ASSERT(first == second);
    ASSERT(first.data() != second.data());

    String copy = first;
    ASSERT(copy == first);
    ASSERT(copy.data() != first.data());

    const char* first_data = first.data();
    String moved = static_cast<String&&>(first);
    ASSERT(moved == "stored string");
    ASSERT(first.empty());
    ASSERT(moved.data() != first_data);
    ASSERT(string_data_is_inline(moved));

    String& alias = moved;
    moved = alias;
    ASSERT(moved == "stored string");

    ASSERT(String("abc") < String("abd"));
    ASSERT(String("ab") < String("abc"));
    ASSERT(!(String("abc") < String("abc")));

    const char embedded_zero[] = {'a', 0, 'b'};
    String embedded(embedded_zero, 3);
    ASSERT(!(embedded == "a"));

    char long_text[300];
    for (char& character : long_text)
        character = 'x';
    long_text[299] = 0;
    String long_string = long_text;
    String long_copy = long_string;
    ASSERT(long_string == long_copy);
    ASSERT(long_string.data() != long_copy.data());

    const char* long_data = long_string.data();
    String long_moved = static_cast<String&&>(long_string);
    ASSERT(long_moved == long_copy);
    ASSERT(long_moved.data() == long_data);
    ASSERT(long_string.empty());

    char boundary_text[sizeof(String) + 1];
    for (char& character : boundary_text)
        character = 'b';
    boundary_text[sizeof(String)] = 0;

    String largest_small(boundary_text, sizeof(String) - 1);
    String smallest_heap(boundary_text, sizeof(String));
    ASSERT(string_data_is_inline(largest_small));
    ASSERT(!string_data_is_inline(smallest_heap));

    String span_string = "span";
    Span<const char> span = span_string;
    ASSERT(span.data == span_string.data());
    ASSERT(span.size == span_string.size());

    std::array<String, 8> thread_strings;
    std::vector<std::jthread> threads;
    for (size_t i = 0; i < thread_strings.size(); i++) {
        threads.emplace_back([i, &thread_strings] {
            thread_strings[i] = "thread string";
        });
    }
    threads.clear(); // joins
    for (size_t i = 1; i < thread_strings.size(); i++) {
        ASSERT(thread_strings[i] == thread_strings[0]);
        ASSERT(thread_strings[i].data() != thread_strings[0].data());
    }
}
