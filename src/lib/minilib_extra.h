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

//
// Sorting
//

/*
    pdqsort.h - Pattern-defeating quicksort.

    Copyright (c) 2021 Orson Peters

    This software is provided 'as-is', without any express or implied warranty. In no event will the
    authors be held liable for any damages arising from the use of this software.

    Permission is granted to anyone to use this software for any purpose, including commercial
    applications, and to alter it and redistribute it freely, subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not claim that you wrote the
       original software. If you use this software in a product, an acknowledgment in the product
       documentation would be appreciated but is not required.

    2. Altered source versions must be plainly marked as such, and must not be misrepresented as
       being the original software.

    3. This notice may not be removed or altered from any source distribution.
*/
// Adapted for minilib: contiguous spans, local helpers and a heapsort fallback
namespace minilib_internal {
    template <typename T>
    struct Partition_Result {
        T* first;
        bool second;
    };

    template <typename T>
    struct Less {
        bool operator()(const T& a, const T& b) const { return a < b; }
    };

    template <typename T>
    void swap_at(T* a, T* b)
    {
        swap_values(*a, *b);
    }

    template <typename T, typename Compare>
    void sift_down(T* data, size_t root, size_t count, Compare comp)
    {
        T value = ::rvalue(data[root]);
        while (root < count / 2) {
            size_t child = root * 2 + 1;
            if (child + 1 < count && comp(data[child], data[child + 1])) {
                child++;
            }
            if (!comp(value, data[child])) {
                break;
            }
            data[root] = ::rvalue(data[child]);
            root = child;
        }
        data[root] = ::rvalue(value);
    }

    template <typename T, typename Compare>
    void heap_sort(T* begin, T* end, Compare comp)
    {
        size_t count = size_t(end - begin);
        for (size_t parent = count / 2; parent != 0;) {
            sift_down(begin, --parent, count, comp);
        }
        while (count > 1) {
            swap_at(begin, begin + --count);
            sift_down(begin, 0, count, comp);
        }
    }

    enum {
        // Partitions below this size are sorted using insertion sort
        insertion_sort_threshold = 24,

        // Partitions above this size use Tukey's ninther to select the pivot
        ninther_threshold = 128,

        // When we detect an already sorted partition, attempt an insertion sort that allows this
        // amount of element moves before giving up
        partial_insertion_sort_limit = 8,

        // Must be multiple of 8 due to loop unrolling, and < 256 to fit in unsigned char
        block_size = 64,

        // Cacheline size, assumes power of two
        cacheline_size = 64

    };


    // Returns floor(log2(n)), assumes n > 0
    template<class T>
    inline int log2(T n) {
        int log = 0;
        while (n >>= 1) ++log;
        return log;
    }

    // Sorts [begin, end) using insertion sort with the given comparison function
    template<class T, class Compare>
    inline void insertion_sort(T* begin, T* end, Compare comp) {
        if (begin == end) {
            return;
        }

        for (T* cur = begin + 1; cur != end; ++cur) {
            T* sift = cur;
            T* sift_1 = cur - 1;

            // Compare first so we can avoid 2 moves for an element already positioned correctly
            if (comp(*sift, *sift_1)) {
                T tmp = ::rvalue(*sift);

                do { *sift-- = ::rvalue(*sift_1); }
                while (sift != begin && comp(tmp, *--sift_1));

                *sift = ::rvalue(tmp);
            }
        }
    }

    // Sorts [begin, end) using insertion sort with the given comparison function. Assumes
    // *(begin - 1) is an element smaller than or equal to any element in [begin, end)
    template<class T, class Compare>
    inline void unguarded_insertion_sort(T* begin, T* end, Compare comp) {
        if (begin == end) {
            return;
        }

        for (T* cur = begin + 1; cur != end; ++cur) {
            T* sift = cur;
            T* sift_1 = cur - 1;

            // Compare first so we can avoid 2 moves for an element already positioned correctly
            if (comp(*sift, *sift_1)) {
                T tmp = ::rvalue(*sift);

                do { *sift-- = ::rvalue(*sift_1); }
                while (comp(tmp, *--sift_1));

                *sift = ::rvalue(tmp);
            }
        }
    }

    // Attempts to use insertion sort on [begin, end). Will return false if more than
    // partial_insertion_sort_limit elements were moved, and abort sorting. Otherwise it will
    // successfully sort and return true
    template<class T, class Compare>
    inline bool partial_insertion_sort(T* begin, T* end, Compare comp) {
        if (begin == end) {
            return true;
        }

        size_t limit = 0;
        for (T* cur = begin + 1; cur != end; ++cur) {
            T* sift = cur;
            T* sift_1 = cur - 1;

            // Compare first so we can avoid 2 moves for an element already positioned correctly
            if (comp(*sift, *sift_1)) {
                T tmp = ::rvalue(*sift);

                do { *sift-- = ::rvalue(*sift_1); }
                while (sift != begin && comp(tmp, *--sift_1));

                *sift = ::rvalue(tmp);
                limit += cur - sift;
            }

            if (limit > partial_insertion_sort_limit) {
                return false;
            }
        }

        return true;
    }

    template<class T, class Compare>
    inline void sort2(T* a, T* b, Compare comp) {
        if (comp(*b, *a)) {
            swap_at(a, b);
        }
    }

    // Sorts the elements *a, *b and *c using comparison function comp
    template<class T, class Compare>
    inline void sort3(T* a, T* b, T* c, Compare comp) {
        sort2(a, b, comp);
        sort2(b, c, comp);
        sort2(a, b, comp);
    }

    template<class T>
    inline T* align_cacheline(T* p) {
        uintptr_t ip = reinterpret_cast<uintptr_t>(p);
        ip = (ip + cacheline_size - 1) & -cacheline_size;
        return reinterpret_cast<T*>(ip);
    }

    template<class T>
    inline void swap_offsets(T* first, T* last,
                             unsigned char* offsets_l, unsigned char* offsets_r,
                             size_t num, bool use_swaps) {
        if (use_swaps) {
            // This case is needed for the descending distribution, where we need
            // to have proper swapping for pdqsort to remain O(n)
            for (size_t i = 0; i < num; ++i) {
                swap_at(first + offsets_l[i], last - offsets_r[i]);
            }
        } else if (num > 0) {
            T* l = first + offsets_l[0]; T* r = last - offsets_r[0];
            T tmp(::rvalue(*l)); *l = ::rvalue(*r);
            for (size_t i = 1; i < num; ++i) {
                l = first + offsets_l[i]; *r = ::rvalue(*l);
                r = last - offsets_r[i]; *l = ::rvalue(*r);
            }
            *r = ::rvalue(tmp);
        }
    }

    // Partitions [begin, end) around pivot *begin using comparison function comp. Elements equal
    // to the pivot are put in the right-hand partition. Returns the position of the pivot after
    // partitioning and whether the passed sequence already was correctly partitioned. Assumes the
    // pivot is a median of at least 3 elements and that [begin, end) is at least
    // insertion_sort_threshold long. Uses branchless partitioning
    template<class T, class Compare>
    inline Partition_Result<T> partition_right_branchless(T* begin, T* end, Compare comp) {

        // Move pivot into local for speed
        T pivot(::rvalue(*begin));
        T* first = begin;
        T* last = end;

        // Find the first element greater than or equal than the pivot (the median of 3 guarantees
        // this exists)
        while (comp(*++first, pivot));

        // Find the first element strictly smaller than the pivot. We have to guard this search if
        // there was no element before *first
        if (first - 1 == begin) {
            while (first < last && !comp(*--last, pivot));
        }
        else {
            while (!comp(*--last, pivot)) {}
        }

        // If the first pair of elements that should be swapped to partition are the same element,
        // the passed in sequence already was correctly partitioned
        bool already_partitioned = first >= last;
        if (!already_partitioned) {
            swap_at(first, last);
            ++first;

            // The following branchless partitioning is derived from "BlockQuicksort: How Branch
            // Mispredictions don’t affect Quicksort" by Stefan Edelkamp and Armin Weiss, but
            // heavily micro-optimized
            unsigned char offsets_l_storage[block_size + cacheline_size];
            unsigned char offsets_r_storage[block_size + cacheline_size];
            unsigned char* offsets_l = align_cacheline(offsets_l_storage);
            unsigned char* offsets_r = align_cacheline(offsets_r_storage);

            T* offsets_l_base = first;
            T* offsets_r_base = last;
            size_t num_l, num_r, start_l, start_r;
            num_l = num_r = start_l = start_r = 0;

            while (first < last) {
                // Fill up offset blocks with elements that are on the wrong side.
                // First we determine how much elements are considered for each offset block
                size_t num_unknown = last - first;
                size_t left_split = num_l == 0 ? (num_r == 0 ? num_unknown / 2 : num_unknown) : 0;
                size_t right_split = num_r == 0 ? (num_unknown - left_split) : 0;

                // Fill the offset blocks
                if (left_split >= block_size) {
                    for (size_t i = 0; i < block_size;) {
                        offsets_l[num_l] = static_cast<unsigned char>(i++); num_l += !comp(*first, pivot); ++first;
                        offsets_l[num_l] = static_cast<unsigned char>(i++); num_l += !comp(*first, pivot); ++first;
                        offsets_l[num_l] = static_cast<unsigned char>(i++); num_l += !comp(*first, pivot); ++first;
                        offsets_l[num_l] = static_cast<unsigned char>(i++); num_l += !comp(*first, pivot); ++first;
                        offsets_l[num_l] = static_cast<unsigned char>(i++); num_l += !comp(*first, pivot); ++first;
                        offsets_l[num_l] = static_cast<unsigned char>(i++); num_l += !comp(*first, pivot); ++first;
                        offsets_l[num_l] = static_cast<unsigned char>(i++); num_l += !comp(*first, pivot); ++first;
                        offsets_l[num_l] = static_cast<unsigned char>(i++); num_l += !comp(*first, pivot); ++first;
                    }
                } else {
                    for (size_t i = 0; i < left_split;) {
                        offsets_l[num_l] = static_cast<unsigned char>(i++); num_l += !comp(*first, pivot); ++first;
                    }
                }

                if (right_split >= block_size) {
                    for (size_t i = 0; i < block_size;) {
                        offsets_r[num_r] = static_cast<unsigned char>(++i); num_r += comp(*--last, pivot);
                        offsets_r[num_r] = static_cast<unsigned char>(++i); num_r += comp(*--last, pivot);
                        offsets_r[num_r] = static_cast<unsigned char>(++i); num_r += comp(*--last, pivot);
                        offsets_r[num_r] = static_cast<unsigned char>(++i); num_r += comp(*--last, pivot);
                        offsets_r[num_r] = static_cast<unsigned char>(++i); num_r += comp(*--last, pivot);
                        offsets_r[num_r] = static_cast<unsigned char>(++i); num_r += comp(*--last, pivot);
                        offsets_r[num_r] = static_cast<unsigned char>(++i); num_r += comp(*--last, pivot);
                        offsets_r[num_r] = static_cast<unsigned char>(++i); num_r += comp(*--last, pivot);
                    }
                } else {
                    for (size_t i = 0; i < right_split;) {
                        offsets_r[num_r] = static_cast<unsigned char>(++i); num_r += comp(*--last, pivot);
                    }
                }

                // Swap elements and update block sizes and first/last boundaries
                size_t num = min(num_l, num_r);
                swap_offsets(offsets_l_base, offsets_r_base,
                             offsets_l + start_l, offsets_r + start_r,
                             num, num_l == num_r);
                num_l -= num; num_r -= num;
                start_l += num; start_r += num;

                if (num_l == 0) {
                    start_l = 0;
                    offsets_l_base = first;
                }

                if (num_r == 0) {
                    start_r = 0;
                    offsets_r_base = last;
                }
            }

            // We have now fully identified [first, last)'s proper position. Swap the last elements
            if (num_l) {
                offsets_l += start_l;
                while (num_l--) swap_at(offsets_l_base + offsets_l[num_l], --last);
                first = last;
            }
            if (num_r) {
                offsets_r += start_r;
                while (num_r--) swap_at(offsets_r_base - offsets_r[num_r], first), ++first;
                last = first;
            }
        }

        // Put the pivot in the right place
        T* pivot_pos = first - 1;
        *begin = ::rvalue(*pivot_pos);
        *pivot_pos = ::rvalue(pivot);

        return Partition_Result<T>{pivot_pos, already_partitioned};
    }



    // Partitions [begin, end) around pivot *begin using comparison function comp. Elements equal
    // to the pivot are put in the right-hand partition. Returns the position of the pivot after
    // partitioning and whether the passed sequence already was correctly partitioned. Assumes the
    // pivot is a median of at least 3 elements and that [begin, end) is at least
    // insertion_sort_threshold long
    template<class T, class Compare>
    inline Partition_Result<T> partition_right(T* begin, T* end, Compare comp) {

        // Move pivot into local for speed
        T pivot(::rvalue(*begin));

        T* first = begin;
        T* last = end;

        // Find the first element greater than or equal than the pivot (the median of 3 guarantees
        // this exists)
        while (comp(*++first, pivot));

        // Find the first element strictly smaller than the pivot. We have to guard this search if
        // there was no element before *first
        if (first - 1 == begin) {
            while (first < last && !comp(*--last, pivot));
        }
        else {
            while (!comp(*--last, pivot)) {}
        }

        // If the first pair of elements that should be swapped to partition are the same element,
        // the passed in sequence already was correctly partitioned
        bool already_partitioned = first >= last;

        // Keep swapping pairs of elements that are on the wrong side of the pivot. Previously
        // swapped pairs guard the searches, which is why the first iteration is special-cased
        // above
        while (first < last) {
            swap_at(first, last);
            while (comp(*++first, pivot));
            while (!comp(*--last, pivot));
        }

        // Put the pivot in the right place
        T* pivot_pos = first - 1;
        *begin = ::rvalue(*pivot_pos);
        *pivot_pos = ::rvalue(pivot);

        return Partition_Result<T>{pivot_pos, already_partitioned};
    }

    // Similar function to the one above, except elements equal to the pivot are put to the left of
    // the pivot and it doesn't check or return if the passed sequence already was partitioned.
    // Since this is rarely used (the many equal case), and in that case pdqsort already has O(n)
    // performance, no block quicksort is applied here for simplicity
    template<class T, class Compare>
    inline T* partition_left(T* begin, T* end, Compare comp) {

        T pivot(::rvalue(*begin));
        T* first = begin;
        T* last = end;

        while (comp(pivot, *--last));

        if (last + 1 == end) {
            while (first < last && !comp(pivot, *++first));
        }
        else {
            while (!comp(pivot, *++first)) {}
        }

        while (first < last) {
            swap_at(first, last);
            while (comp(pivot, *--last));
            while (!comp(pivot, *++first));
        }

        T* pivot_pos = last;
        *begin = ::rvalue(*pivot_pos);
        *pivot_pos = ::rvalue(pivot);

        return pivot_pos;
    }


    template<class T, class Compare, bool Branchless>
    inline void pdqsort_loop(T* begin, T* end, Compare comp, int bad_allowed, bool leftmost = true) {
        typedef ptrdiff_t diff_t;

        // Use a while loop for tail recursion elimination
        while (true) {
            diff_t size = end - begin;

            // Insertion sort is faster for small arrays
            if (size < insertion_sort_threshold) {
                if (leftmost) {
                    insertion_sort(begin, end, comp);
                }
                else {
                    unguarded_insertion_sort(begin, end, comp);
                }
                return;
            }

            // Choose pivot as median of 3 or pseudomedian of 9
            diff_t s2 = size / 2;
            if (size > ninther_threshold) {
                sort3(begin, begin + s2, end - 1, comp);
                sort3(begin + 1, begin + (s2 - 1), end - 2, comp);
                sort3(begin + 2, begin + (s2 + 1), end - 3, comp);
                sort3(begin + (s2 - 1), begin + s2, begin + (s2 + 1), comp);
                swap_at(begin, begin + s2);
            } else {
                sort3(begin + s2, begin, end - 1, comp);
            }

            // If *(begin - 1) is the end of the right partition of a previous partition operation
            // there is no element in [begin, end) that is smaller than *(begin - 1). Then if our
            // pivot compares equal to *(begin - 1) we change strategy, putting equal elements in
            // the left partition, greater elements in the right partition. We do not have to
            // recurse on the left partition, since it's sorted (all equal)
            if (!leftmost && !comp(*(begin - 1), *begin)) {
                begin = partition_left(begin, end, comp) + 1;
                continue;
            }

            // Partition and get results
            Partition_Result<T> part_result =
                Branchless ? partition_right_branchless(begin, end, comp)
                           : partition_right(begin, end, comp);
            T* pivot_pos = part_result.first;
            bool already_partitioned = part_result.second;

            // Check for a highly unbalanced partition
            diff_t l_size = pivot_pos - begin;
            diff_t r_size = end - (pivot_pos + 1);
            bool highly_unbalanced = l_size < size / 8 || r_size < size / 8;

            // If we got a highly unbalanced partition we shuffle elements to break many patterns
            if (highly_unbalanced) {
                // If we had too many bad partitions, switch to heapsort to guarantee O(n log n)
                if (--bad_allowed == 0) {
                    heap_sort(begin, end, comp);
                    return;
                }

                if (l_size >= insertion_sort_threshold) {
                    swap_at(begin,             begin + l_size / 4);
                    swap_at(pivot_pos - 1, pivot_pos - l_size / 4);

                    if (l_size > ninther_threshold) {
                        swap_at(begin + 1,         begin + (l_size / 4 + 1));
                        swap_at(begin + 2,         begin + (l_size / 4 + 2));
                        swap_at(pivot_pos - 2, pivot_pos - (l_size / 4 + 1));
                        swap_at(pivot_pos - 3, pivot_pos - (l_size / 4 + 2));
                    }
                }

                if (r_size >= insertion_sort_threshold) {
                    swap_at(pivot_pos + 1, pivot_pos + (1 + r_size / 4));
                    swap_at(end - 1,                   end - r_size / 4);

                    if (r_size > ninther_threshold) {
                        swap_at(pivot_pos + 2, pivot_pos + (2 + r_size / 4));
                        swap_at(pivot_pos + 3, pivot_pos + (3 + r_size / 4));
                        swap_at(end - 2,             end - (1 + r_size / 4));
                        swap_at(end - 3,             end - (2 + r_size / 4));
                    }
                }
            } else {
                // If we were decently balanced and we tried to sort an already partitioned
                // sequence try to use insertion sort
                if (already_partitioned && partial_insertion_sort(begin, pivot_pos, comp)
                                        && partial_insertion_sort(pivot_pos + 1, end, comp)) {
                    return;
                }
            }

            // Sort the left partition first using recursion and do tail recursion elimination for
            // the right-hand partition
            pdqsort_loop<T, Compare, Branchless>(begin, pivot_pos, comp, bad_allowed, leftmost);
            begin = pivot_pos + 1;
            leftmost = false;
        }
    }

    template <bool ConstructScratch, typename T, typename Compare>
    void merge_sort(T* data, size_t count, T* scratch, Compare& less)
    {
        if (count <= 24) {
            if (count > 1) {
                insertion_sort(data, data + count, less);
            }
            return;
        }
        size_t middle = count / 2;
        merge_sort<ConstructScratch>(data, middle, scratch, less);
        merge_sort<ConstructScratch>(data + middle, count - middle, scratch, less);
        if (!less(data[middle], data[middle - 1])) {
            return;
        }
        for (size_t i = 0; i < middle; i++) {
            if constexpr (ConstructScratch) {
                ::new (Placement_New{}, scratch + i) T(::rvalue(data[i]));
            } else {
                scratch[i] = ::rvalue(data[i]);
            }
        }
        size_t left = 0, right = middle, out = 0;
        while (left < middle && right < count) {
            if (less(data[right], scratch[left])) {
                data[out++] = ::rvalue(data[right++]);
            } else {
                data[out++] = ::rvalue(scratch[left++]);
            }
        }
        while (left < middle) {
            data[out++] = ::rvalue(scratch[left++]);
        }
        if constexpr (ConstructScratch && !has_trivial_destructor<T>()) {
            for (size_t i = middle; i > 0; i--) {
                scratch[i - 1].~T();
            }
        }
    }
} // namespace minilib_internal

// Unstable PDQsort, O(N log N) worst case, without heap allocation.
// Branchless partitioning suits cheap comparisons; use sort<false> for expensive comparators.
// All sort overloads require a strict weak ordering and nonthrowing comparisons and moves.
// Floating-point comparisons must order NaNs explicitly if inputs can contain them
//
template <bool Branchless = true, typename T, typename Compare>
void sort(Span<T> values, Compare less)
requires requires(Compare& compare, T& a, T& b) { bool(compare(a, b)); }
{
    if (values.size < 2) {
        return;
    }
    if (values.size <= 32) {
        minilib_internal::insertion_sort(values.data, values.data + values.size, less);
    } else {
        minilib_internal::pdqsort_loop<T, Compare, Branchless>(values.data, values.data + values.size, less,
            minilib_internal::log2(values.size));
    }
}

template <bool Branchless = true, typename T>
void sort(Span<T> values)
{
    sort<Branchless>(values, minilib_internal::Less<T>{});
}

// Stable mergesort with O(N log N) worst-case work.
// For N > 24, scratch must contain at least N/2 live objects, disjoint from values.
// Scratch contents are overwritten and may be moved from on return
//
template <typename T, typename Compare>
void stable_sort(Span<T> values, Span<T> scratch, Compare less)
requires requires(Compare& compare, T& a, T& b) { bool(compare(a, b)); }
{
    ASSERT(values.size <= 24 || scratch.size >= values.size / 2);
    minilib_internal::merge_sort<false>(values.data, values.size, scratch.data, less);
}

template <typename T>
void stable_sort(Span<T> values, Span<T> scratch)
{
    stable_sort(values, scratch, minilib_internal::Less<T>{});
}

// Allocates N/2 temporary slots for N > 24; T need not be default constructible
//
template <typename T, typename Compare>
void stable_sort(Span<T> values, Compare less)
requires requires(Compare& compare, T& a, T& b) { bool(compare(a, b)); }
{
    if (values.size <= 24) {
        minilib_internal::merge_sort<false>(values.data, values.size, static_cast<T*>(nullptr), less);
        return;
    }
    T* scratch = static_cast<T*>(minilib_internal::allocate_array_storage(values.size / 2, sizeof(T), alignof(T)));
    minilib_internal::merge_sort<true>(values.data, values.size, scratch, less);
    minilib_internal::free_array_storage(scratch, alignof(T));
}

template <typename T>
void stable_sort(Span<T> values)
{
    stable_sort(values, minilib_internal::Less<T>{});
}
