#pragma once

struct Memory_Pool {
    void* pool_ptr = nullptr;
    size_t pool_size = 0;
    size_t allocated_bytes = 0;

    ~Memory_Pool()
    {
        ASSERT(pool_ptr == nullptr);
    }

    void allocate_pool_memory(size_t size)
    {
        ASSERT(pool_ptr == nullptr);
        pool_ptr = (uint8_t*)allocate_aligned_memory(size, 64);
        pool_size = size;
    }

    void deallocate_pool_memory()
    {
        free_aligned_memory(pool_ptr);
        pool_ptr = nullptr;
        pool_size = 0;
        allocated_bytes = 0;
    }

    void reset()
    {
        allocated_bytes = 0;
    }

    template <typename T>
    void* allocate()
    {
        constexpr size_t mask = alignof(T) - 1;
        size_t offset = (allocated_bytes + mask) & ~mask;
        size_t new_size = offset + sizeof(T);

        ASSERT(new_size <= pool_size);
        if (new_size > pool_size)
            return nullptr;
        allocated_bytes = new_size;

        void* ptr = static_cast<uint8_t*>(pool_ptr) + offset;
        ASSERT((reinterpret_cast<uintptr_t>(ptr) & mask) == 0);
        return ptr;
    }
};
