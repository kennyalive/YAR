#pragma once

#include "../vk.h"

struct Apply_Tone_Mapping {
    VkPipeline pipeline;

    void create(uint32_t output_image_heap_offset, uint32_t tonemapped_image_heap_offset);
    void destroy();
    void dispatch();
};
