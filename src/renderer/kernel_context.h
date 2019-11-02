#pragma once

#include "vk.h"

struct Kernel_Context {
    VkDescriptorSetLayout light_descriptor_set_layout = VK_NULL_HANDLE;
    VkDescriptorSetLayout material_descriptor_set_layout = VK_NULL_HANDLE;
    VkDescriptorSetLayout image_descriptor_set_layout = VK_NULL_HANDLE;
};

