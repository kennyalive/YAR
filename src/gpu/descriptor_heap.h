#pragma once

#include "vk.h"

struct Descriptor_Heap
{
    Vk_Buffer resource_heap_buffer;
    uint32_t resource_reserved_region_offset = 0;

    Vk_Buffer sampler_heap_buffer;
    uint32_t sampler_reserved_region_offset = 0;

    void create(uint32_t descriptor_data_size);
    void destroy();
    void bind(VkCommandBuffer command_buffer) const;

    void write_image_descriptor(VkImage image, VkFormat image_format, VkDescriptorType descriptor_type, uint32_t heap_offset) const;
    void write_buffer_descriptor(VkDeviceAddressRangeEXT address_range, VkDescriptorType descriptor_type, uint32_t heap_offset);
    void write_acceleration_structure_descriptor(VkDeviceAddress device_address, uint32_t heap_offset);
    void write_sampler_descriptor(const VkSamplerCreateInfo& sampler_ci, uint32_t heap_offset);
};
