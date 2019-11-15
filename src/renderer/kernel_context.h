#pragma once

#include "vk.h"

struct Kernel_Context {
    VkDescriptorSetLayout base_descriptor_set_layout = VK_NULL_HANDLE;
    VkDescriptorSetLayout light_descriptor_set_layout = VK_NULL_HANDLE;
    VkDescriptorSetLayout material_descriptor_set_layout = VK_NULL_HANDLE;
};

// In order to use benefits of pipeline layout compatibility we need to
// make sure that all compatible pipeline layouts have the same configuration
// of push constants.
constexpr int Compatible_Layout_Push_Constant_Count = 8;

