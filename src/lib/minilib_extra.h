#pragma once

#include "minilib.h"

template <typename T, typename U>
bool contains(Span<T> values, const U& value)
{
    for (const T& element : values) {
        if (element == value) {
            return true;
        }
    }
    return false;
}

// Returns the index of the first element >= value, or values.size if none exists.
// Values must be sorted in ascending order
template <typename T, typename U>
size_t find_first_greater_or_equal(Span<T> sorted_values, const U& value)
{
    size_t first = 0;
    size_t last = sorted_values.size;

    while (first < last) {
        size_t middle = first + (last - first) / 2;
        if (sorted_values[middle] < value) {
            first = middle + 1;
        } else {
            last = middle;
        }
    }
    return first;
}

template <typename T>
void reverse(Span<T> values)
{
    for (size_t i = 0; i < values.size / 2; i++) {
        size_t j = values.size - 1 - i;
        swap_values(values[i], values[j]);
    }
}
