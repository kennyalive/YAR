#pragma once

#include "../vk.h"

struct Copy_To_Swapchain {
    VkPipeline pipeline;

    void create(uint32_t tonemapped_image_heap_offset, uint32_t swapchain_images_heap_offset);
    void destroy();
    void dispatch();
};
