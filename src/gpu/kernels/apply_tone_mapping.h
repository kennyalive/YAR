#pragma once

#include "../vk.h"

struct Global_Descriptors;

struct Apply_Tone_Mapping {
    VkPipeline pipeline;

    void create(Global_Descriptors& global_descriptors);
    void destroy();
    void dispatch();
};
