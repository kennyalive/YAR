#pragma once

struct Descriptor_Heap;

constexpr uint32_t max_swapchain_images = 8;

struct Descriptors {
    uint32_t output_image = 0;
    uint32_t swapchain_images = 0;

    uint32_t image_descriptor_size = 0;

    void initialize(Descriptor_Heap& descriptor_heap);
};
