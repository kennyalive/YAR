#pragma once

#include "minilib.h"

#ifndef MINILIB_HASH_MAP_USE_SSE2
#if defined(_M_X64) || defined(__SSE2__)
#define MINILIB_HASH_MAP_USE_SSE2 1
#else
#define MINILIB_HASH_MAP_USE_SSE2 0
#endif
#endif

#if MINILIB_HASH_MAP_USE_SSE2
#include <emmintrin.h> // SSE2 intrinsics
#endif

#if defined(_MSC_VER)
#include <intrin0.h> // _BitScanForward
#endif

#if MINILIB_USE_NAMESPACE
namespace mini {
#endif

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
// Upstream: https://github.com/orlp/pdqsort/blob/b1ef26a55cdb60d236a5cb199c4234c704f46726/pdqsort.h
// Adapted for minilib: contiguous spans, local helpers and a heapsort fallback
namespace mini_internal {
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
        T value = rvalue(data[root]);
        while (root < count / 2) {
            size_t child = root * 2 + 1;
            if (child + 1 < count && comp(data[child], data[child + 1])) {
                child++;
            }
            if (!comp(value, data[child])) {
                break;
            }
            data[root] = rvalue(data[child]);
            root = child;
        }
        data[root] = rvalue(value);
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
    int log2(T n) {
        int log = 0;
        while (n >>= 1) ++log;
        return log;
    }

    // Sorts [begin, end) using insertion sort with the given comparison function
    template<class T, class Compare>
    void insertion_sort(T* begin, T* end, Compare comp) {
        if (begin == end) {
            return;
        }

        for (T* cur = begin + 1; cur != end; ++cur) {
            T* sift = cur;
            T* sift_1 = cur - 1;

            // Compare first so we can avoid 2 moves for an element already positioned correctly
            if (comp(*sift, *sift_1)) {
                T tmp = rvalue(*sift);

                do { *sift-- = rvalue(*sift_1); }
                while (sift != begin && comp(tmp, *--sift_1));

                *sift = rvalue(tmp);
            }
        }
    }

    // Sorts [begin, end) using insertion sort with the given comparison function. Assumes
    // *(begin - 1) is an element smaller than or equal to any element in [begin, end)
    template<class T, class Compare>
    void unguarded_insertion_sort(T* begin, T* end, Compare comp) {
        if (begin == end) {
            return;
        }

        for (T* cur = begin + 1; cur != end; ++cur) {
            T* sift = cur;
            T* sift_1 = cur - 1;

            // Compare first so we can avoid 2 moves for an element already positioned correctly
            if (comp(*sift, *sift_1)) {
                T tmp = rvalue(*sift);

                do { *sift-- = rvalue(*sift_1); }
                while (comp(tmp, *--sift_1));

                *sift = rvalue(tmp);
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
                T tmp = rvalue(*sift);

                do { *sift-- = rvalue(*sift_1); }
                while (sift != begin && comp(tmp, *--sift_1));

                *sift = rvalue(tmp);
                limit += cur - sift;
            }

            if (limit > partial_insertion_sort_limit) {
                return false;
            }
        }

        return true;
    }

    template<class T, class Compare>
    void sort2(T* a, T* b, Compare comp) {
        if (comp(*b, *a)) {
            swap_at(a, b);
        }
    }

    // Sorts the elements *a, *b and *c using comparison function comp
    template<class T, class Compare>
    void sort3(T* a, T* b, T* c, Compare comp) {
        sort2(a, b, comp);
        sort2(b, c, comp);
        sort2(a, b, comp);
    }

    template<class T>
    T* align_cacheline(T* p) {
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
            T tmp(rvalue(*l)); *l = rvalue(*r);
            for (size_t i = 1; i < num; ++i) {
                l = first + offsets_l[i]; *r = rvalue(*l);
                r = last - offsets_r[i]; *l = rvalue(*r);
            }
            *r = rvalue(tmp);
        }
    }

    // Partitions [begin, end) around pivot *begin using comparison function comp. Elements equal
    // to the pivot are put in the right-hand partition. Returns the position of the pivot after
    // partitioning and whether the passed sequence already was correctly partitioned. Assumes the
    // pivot is a median of at least 3 elements and that [begin, end) is at least
    // insertion_sort_threshold long. Uses branchless partitioning
    template<class T, class Compare>
    Partition_Result<T> partition_right_branchless(T* begin, T* end, Compare comp) {

        // Move pivot into local for speed
        T pivot(rvalue(*begin));
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
        *begin = rvalue(*pivot_pos);
        *pivot_pos = rvalue(pivot);

        return Partition_Result<T>{pivot_pos, already_partitioned};
    }



    // Partitions [begin, end) around pivot *begin using comparison function comp. Elements equal
    // to the pivot are put in the right-hand partition. Returns the position of the pivot after
    // partitioning and whether the passed sequence already was correctly partitioned. Assumes the
    // pivot is a median of at least 3 elements and that [begin, end) is at least
    // insertion_sort_threshold long
    template<class T, class Compare>
    Partition_Result<T> partition_right(T* begin, T* end, Compare comp) {

        // Move pivot into local for speed
        T pivot(rvalue(*begin));

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
        *begin = rvalue(*pivot_pos);
        *pivot_pos = rvalue(pivot);

        return Partition_Result<T>{pivot_pos, already_partitioned};
    }

    // Similar function to the one above, except elements equal to the pivot are put to the left of
    // the pivot and it doesn't check or return if the passed sequence already was partitioned.
    // Since this is rarely used (the many equal case), and in that case pdqsort already has O(n)
    // performance, no block quicksort is applied here for simplicity
    template<class T, class Compare>
    T* partition_left(T* begin, T* end, Compare comp) {

        T pivot(rvalue(*begin));
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
        *begin = rvalue(*pivot_pos);
        *pivot_pos = rvalue(pivot);

        return pivot_pos;
    }


    template<class T, class Compare, bool Branchless>
    void pdqsort_loop(T* begin, T* end, Compare comp, int bad_allowed, bool leftmost = true) {
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
                ::new (Placement_New{}, scratch + i) T(rvalue(data[i]));
            } else {
                scratch[i] = rvalue(data[i]);
            }
        }
        size_t left = 0, right = middle, out = 0;
        while (left < middle && right < count) {
            if (less(data[right], scratch[left])) {
                data[out++] = rvalue(data[right++]);
            } else {
                data[out++] = rvalue(scratch[left++]);
            }
        }
        while (left < middle) {
            data[out++] = rvalue(scratch[left++]);
        }
        if constexpr (ConstructScratch && !has_trivial_destructor<T>()) {
            for (size_t i = middle; i > 0; i--) {
                scratch[i - 1].~T();
            }
        }
    }
} // namespace mini_internal

// Unstable PDQsort, O(N log N) worst case, without heap allocation.
// Branchless partitioning suits cheap comparisons; use sort<false> for expensive comparators.
// All sort overloads require a strict weak ordering.
// Floating-point comparisons must order NaNs explicitly if inputs can contain them
template <bool Branchless = true, typename T, typename Compare>
void sort(Span<T> values, Compare less)
{
    if (values.count < 2) {
        return;
    }
    if (values.count <= 32) {
        mini_internal::insertion_sort(values.elements, values.elements + values.count, less);
    } else {
        mini_internal::pdqsort_loop<T, Compare, Branchless>(values.elements, values.elements + values.count, less,
            mini_internal::log2(values.count));
    }
}

template <bool Branchless = true, typename T>
void sort(Span<T> values)
{
    sort<Branchless>(values, mini_internal::Less<T>{});
}

// Stable mergesort with O(N log N) worst-case work.
// For N > 24, scratch must contain at least N/2 live objects, disjoint from values.
// Scratch contents are overwritten and may be moved from on return
template <typename T, typename Compare>
void stable_sort(Span<T> values, Span<T> scratch, Compare less)
{
    ASSERT(values.count <= 24 || scratch.count >= values.count / 2);
    mini_internal::merge_sort<false>(values.elements, values.count, scratch.elements, less);
}

template <typename T>
void stable_sort(Span<T> values, Span<T> scratch)
{
    stable_sort(values, scratch, mini_internal::Less<T>{});
}

// Allocates N/2 temporary slots for N > 24; T need not be default constructible
template <typename T, typename Compare>
void stable_sort(Span<T> values, Compare less)
    // Prevents stable_sort<T>(values, scratch_array) from picking this overload
    // over the scratch overload, which needs a conversion to Span<T>
    requires requires(T& a, T& b) { bool(less(a, b)); }
{
    if (values.count <= 24) {
        mini_internal::merge_sort<false>(values.elements, values.count, static_cast<T*>(nullptr), less);
        return;
    }
    T* scratch = allocate_storage<T>(values.count / 2);
    mini_internal::merge_sort<true>(values.elements, values.count, scratch, less);
    free_storage(scratch);
}

template <typename T>
void stable_sort(Span<T> values)
{
    stable_sort(values, mini_internal::Less<T>{});
}

//
// Hash map
//

// Swiss-table-style grouped probing: https://abseil.io/about/design/swisstables

// Keep probing in the lookup and insertion callers
#if defined(_MSC_VER)
#define MINILIB_FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define MINILIB_FORCE_INLINE inline __attribute__((always_inline))
#else
#define MINILIB_FORCE_INLINE inline
#endif

namespace mini_internal {
    inline uint32_t hash_group_matches(const uint8_t* group, uint8_t value)
    {
#if MINILIB_HASH_MAP_USE_SSE2
        __m128i bytes = _mm_loadu_si128(reinterpret_cast<const __m128i*>(group));
        return uint32_t(_mm_movemask_epi8(_mm_cmpeq_epi8(bytes, _mm_set1_epi8(char(value)))));
#else
        uint32_t mask = 0;
        for (uint32_t i = 0; i < 16; i++) {
            mask |= uint32_t(group[i] == value) << i;
        }
        return mask;
#endif
    }

    inline size_t hash_first_bit(uint32_t mask)
    {
        ASSERT(mask);
#if defined(_MSC_VER)
        unsigned long index;
        _BitScanForward(&index, mask);
        return index;
#else
        return size_t(__builtin_ctz(mask));
#endif
    }

    template <typename Map, typename Entry>
    struct Hash_Map_Iterator
    {
        Map* map;
        size_t index;

        Hash_Map_Iterator(Map* map, size_t index) : map(map), index(index) { skip_empty(); }
        Entry& operator*() const { return map->entries[index]; }
        Hash_Map_Iterator& operator++()
        {
            index++;
            skip_empty();
            return *this;
        }
        bool operator!=(const Hash_Map_Iterator& other) const { return map != other.map || index != other.index; }

    private:
        void skip_empty()
        {
            while (index < map->capacity && !map->occupied(index)) {
                index++;
            }
        }
    };
} // namespace mini_internal

// Flat storage with 16-slot groups and a maximum load of 7/8.
// Insertion can invalidate pointers, references and iterators. Keys must not be modified
// Hash must distribute bits uniformly; equal keys must have equal hashes.
template <typename K, typename V, typename Hash = Hasher>
struct Hash_Map
{
    struct Entry
    {
        K key;
        V value;
    };
    using Iterator = mini_internal::Hash_Map_Iterator<Hash_Map, Entry>;
    using Const_Iterator = mini_internal::Hash_Map_Iterator<const Hash_Map, const Entry>;

    Hash_Map() = default;
    Hash_Map(const Hash_Map&) = delete;
    Hash_Map& operator=(const Hash_Map&) = delete;
    Hash_Map(Hash_Map&& other) noexcept { swap(other); }
    Hash_Map& operator=(Hash_Map&& other) noexcept
    {
        Hash_Map moved(rvalue(other));
        swap(moved);
        return *this;
    }
    ~Hash_Map()
    {
        clear();
        free_storage(entries);
        delete[] metadata;
    }

    size_t size() const { return element_count; }
    bool empty() const { return element_count == 0; }
    Iterator begin() { return {this, 0}; }
    Iterator end() { return {this, capacity}; }
    Const_Iterator begin() const { return {this, 0}; }
    Const_Iterator end() const { return {this, capacity}; }

    // Returns nullptr when the key is absent
    V* find(const K& key)
    {
        Lookup result = locate(entries, metadata, capacity, key, hasher(key));
        return result.found ? &entries[result.index].value : nullptr;
    }
    MINILIB_FORCE_INLINE const V* find(const K& key) const
    {
        Lookup result = locate(entries, metadata, capacity, key, hasher(key));
        return result.found ? &entries[result.index].value : nullptr;
    }
    bool contains(const K& key) const { return find(key) != nullptr; }

    // Returns the existing value, or inserts V{} when the key is absent
    MINILIB_FORCE_INLINE V& get_or_insert(const K& key)
    {
        uint64_t hash = hasher(key);
        Lookup result = locate(entries, metadata, capacity, key, hash);
        if (result.found) {
            return entries[result.index].value;
        }
        // Copy the key before growth in case it refers to an existing entry
        Entry entry{key, V{}};
        return add_new(rvalue(entry), hash, result.index)->value;
    }
    // Leaves the existing value unchanged when the key is already present
    MINILIB_FORCE_INLINE void insert(Entry entry)
    {
        uint64_t hash = hasher(entry.key);
        Lookup result = locate(entries, metadata, capacity, entry.key, hash);
        if (!result.found) {
            add_new(rvalue(entry), hash, result.index);
        }
    }
    // Retains allocated storage for reuse
    void clear()
    {
        if (!capacity) {
            return;
        }
        if constexpr (!has_trivial_destructor<Entry>()) {
            for (size_t i = 0; i < capacity; i++) {
                if (occupied(i)) {
                    entries[i].~Entry();
                }
            }
        }
        memset(metadata, empty_byte, capacity);
        element_count = 0;
    }

private:
    template <typename, typename> friend struct mini_internal::Hash_Map_Iterator;
    struct Lookup
    {
        size_t index;
        bool found;
    };
    static constexpr uint8_t empty_byte = 0x80;
    Entry* entries = nullptr;
    uint8_t* metadata = nullptr;
    size_t capacity = 0;
    size_t element_count = 0;
    Hash hasher{};

    bool occupied(size_t index) const { return metadata[index] != empty_byte; }

    static MINILIB_FORCE_INLINE Lookup locate(const Entry* entries, const uint8_t* metadata, size_t capacity, const K& key, uint64_t hash)
    {
        using mini_internal::hash_group_matches;
        using mini_internal::hash_first_bit;
        if (!capacity) {
            return {0, false};
        }
        size_t group_mask = capacity / 16 - 1;
        size_t group = size_t(hash >> 7) & group_mask;
        size_t step = 0;
        uint8_t tag = uint8_t(hash & 127);
        for (;;) {
            size_t start = group * 16;
            uint32_t candidates = hash_group_matches(metadata + start, tag);
            while (candidates) {
                size_t index = start + hash_first_bit(candidates);
                if (entries[index].key == key) {
                    return {index, true};
                }
                candidates &= candidates - 1;
            }
            uint32_t empty = hash_group_matches(metadata + start, empty_byte);
            if (empty) {
                return {start + hash_first_bit(empty), false};
            }
            // Triangular steps visit every group in a power-of-two table
            group = (group + ++step) & group_mask;
        }
    }
    Entry* place(size_t index, Entry&& entry, uint64_t hash)
    {
        ::new (Placement_New{}, entries + index) Entry(rvalue(entry));
        metadata[index] = uint8_t(hash & 127);
        return entries + index;
    }
    MINILIB_FORCE_INLINE Entry* add_new(Entry&& entry, uint64_t hash, size_t index)
    {
        if (element_count == capacity - capacity / 8) {
            grow();
            index = locate(entries, metadata, capacity, entry.key, hash).index;
        }
        Entry* inserted = place(index, rvalue(entry), hash);
        element_count++;
        return inserted;
    }
    void grow()
    {
        const size_t max_capacity = SIZE_MAX / sizeof(Entry);
        if (capacity > max_capacity / 2 || max_capacity < 16) {
            abort();
        }
        Entry* old_entries = entries;
        uint8_t* old_metadata = metadata;
        size_t old_capacity = capacity;
        capacity = capacity ? capacity * 2 : 16;
        entries = allocate_storage<Entry>(capacity);
        metadata = new uint8_t[capacity];
        memset(metadata, empty_byte, capacity);
        for (size_t i = 0; i < old_capacity; i++) {
            if (old_metadata[i] != empty_byte) {
                uint64_t hash = hasher(old_entries[i].key);
                size_t destination = locate(entries, metadata, capacity, old_entries[i].key, hash).index;
                place(destination, rvalue(old_entries[i]), hash);
                old_entries[i].~Entry();
            }
        }
        free_storage(old_entries);
        delete[] old_metadata;
    }
    void swap(Hash_Map& other) noexcept
    {
        swap_values(entries, other.entries);
        swap_values(metadata, other.metadata);
        swap_values(capacity, other.capacity);
        swap_values(element_count, other.element_count);
        swap_values(hasher, other.hasher);
    }
};

#undef MINILIB_FORCE_INLINE

#if MINILIB_USE_NAMESPACE
} // namespace mini
#endif
