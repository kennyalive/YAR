#pragma once

struct Memory_Pool {
    uint8_t* memory = nullptr;
    int64_t allocated_size = 0;
    int64_t used_size = 0;

    void allocate_pool_memory(int64_t size) {
        ASSERT(memory == nullptr);
        memory = new uint8_t[size];
        allocated_size = size;
    }

    void deallocate_pool_memory() {
        delete[] memory;
        memory = nullptr;
    }

    void reset() {
        used_size = 0;
    }

    template <typename T>
    void* allocate() {
        int64_t offset = (used_size + alignof(T) - 1) & ~(alignof(T) - 1);
        int64_t new_used_size = offset + sizeof(T);

        ASSERT(new_used_size <= allocated_size);
        if (new_used_size > allocated_size)
            return nullptr;

        used_size = new_used_size;
        return memory + offset;
    }
};
