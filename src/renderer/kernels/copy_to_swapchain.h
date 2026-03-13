#pragma once

#include "../vk.h"

struct Descriptors;

struct Copy_To_Swapchain {
    VkPipeline pipeline;

    void create(const Descriptors& descriptors);
    void destroy();
    void dispatch();
};
