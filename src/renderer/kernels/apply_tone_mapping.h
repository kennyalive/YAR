#pragma once

#include "../vk.h"

struct Descriptors;

struct Apply_Tone_Mapping {
    VkPipeline pipeline;

    void create(Descriptors& descriptors);
    void destroy();
    void dispatch();
};
