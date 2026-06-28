#pragma once

#include "../vk.h"

struct Global_Descriptors;

struct Copy_To_Swapchain {
    VkPipeline pipeline;

    void create(const Global_Descriptors& descriptors);
    void destroy();
    void dispatch();
};
