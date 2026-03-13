#include "std.h"
#include "lib/common.h"
#include "descriptor_heap.h"

#include "lib/math.h"

void Descriptor_Heap::create(const VkPhysicalDeviceDescriptorHeapPropertiesEXT& descriptor_heap_properties)
{
    properties = descriptor_heap_properties;

    const uint32_t resource_heap_size = 1024 * 1024;
    ASSERT(resource_heap_size < properties.maxResourceHeapSize);
    resource_heap_buffer = vk_create_mapped_buffer_with_alignment(
        resource_heap_size,
        VK_BUFFER_USAGE_DESCRIPTOR_HEAP_BIT_EXT,
        (uint32_t)properties.resourceHeapAlignment,
        "resource_heap"
    );
    const uint32_t resource_reserved_region_alignment = (uint32_t)std::max(
        properties.bufferDescriptorAlignment,
        properties.imageDescriptorAlignment
    );
    resource_reserved_region_offset = round_up(
        resource_heap_size - (uint32_t)properties.minResourceHeapReservedRange - (resource_reserved_region_alignment - 1),
        resource_reserved_region_alignment
    );

    const uint32_t sampler_heap_size = 32 * 1024;
    ASSERT(sampler_heap_size < properties.maxSamplerHeapSize);
    sampler_heap_buffer = vk_create_mapped_buffer_with_alignment(
        sampler_heap_size,
        VK_BUFFER_USAGE_DESCRIPTOR_HEAP_BIT_EXT,
        (uint32_t)properties.samplerHeapAlignment,
        "sampler_heap"
    );
    const uint32_t sampler_reserved_region_alignment = (uint32_t)properties.samplerDescriptorAlignment;
    sampler_reserved_region_offset = round_up(
        sampler_heap_size - (uint32_t)properties.minSamplerHeapReservedRange - (sampler_reserved_region_alignment - 1),
        sampler_reserved_region_alignment
    );
}

void Descriptor_Heap::destroy()
{
    resource_heap_buffer.destroy();
    sampler_heap_buffer.destroy();
    *this = {};
}

void Descriptor_Heap::bind() const
{
    VkBindHeapInfoEXT resource_heap_info{ VK_STRUCTURE_TYPE_BIND_HEAP_INFO_EXT };
    resource_heap_info.heapRange.address = resource_heap_buffer.device_address;
    resource_heap_info.heapRange.size = resource_heap_buffer.size;
    resource_heap_info.reservedRangeOffset = resource_reserved_region_offset;
    resource_heap_info.reservedRangeSize = properties.minResourceHeapReservedRange;
    vkCmdBindResourceHeapEXT(vk.command_buffer, &resource_heap_info);

    VkBindHeapInfoEXT sampler_heap_info{ VK_STRUCTURE_TYPE_BIND_HEAP_INFO_EXT };
    sampler_heap_info.heapRange.address = sampler_heap_buffer.device_address;
    sampler_heap_info.heapRange.size = sampler_heap_buffer.size;
    sampler_heap_info.reservedRangeOffset = sampler_reserved_region_offset;
    sampler_heap_info.reservedRangeSize = properties.minSamplerHeapReservedRange;
    vkCmdBindSamplerHeapEXT(vk.command_buffer, &sampler_heap_info);
}

uint32_t Descriptor_Heap::allocate_buffer_descriptor()
{
    const uint32_t descriptor_offset = round_up(current_resource_heap_offset, (uint32_t)properties.bufferDescriptorAlignment);
    current_resource_heap_offset = descriptor_offset + (uint32_t)properties.bufferDescriptorSize;
    ASSERT(current_resource_heap_offset <= resource_reserved_region_offset);
    return descriptor_offset;
}

uint32_t Descriptor_Heap::allocate_image_descriptor(uint32_t count)
{
    const uint32_t descriptor_offset = round_up(current_resource_heap_offset, (uint32_t)properties.imageDescriptorAlignment);
    current_resource_heap_offset = descriptor_offset + (uint32_t)properties.imageDescriptorSize * count;
    ASSERT(current_resource_heap_offset <= resource_reserved_region_offset);
    return descriptor_offset;
}

uint32_t Descriptor_Heap::allocate_sampler_descriptor()
{
    const uint32_t descriptor_offset = round_up(current_sampler_heap_offset, (uint32_t)properties.samplerDescriptorAlignment);
    current_sampler_heap_offset = descriptor_offset + (uint32_t)properties.samplerDescriptorSize;
    ASSERT(current_sampler_heap_offset <= sampler_reserved_region_offset);
    return descriptor_offset;
}
