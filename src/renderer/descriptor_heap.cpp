#include "std.h"
#include "lib/common.h"
#include "descriptor_heap.h"

#include "lib/math.h"

void Descriptor_Heap::create(const VkPhysicalDeviceDescriptorHeapPropertiesEXT& descriptor_heap_properties)
{
    properties = descriptor_heap_properties;

    const uint32_t resource_heap_size = 1024 * 1024;
    ASSERT(resource_heap_size <= properties.maxResourceHeapSize);
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
    ASSERT(sampler_heap_size <= properties.maxSamplerHeapSize);
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

void Descriptor_Heap::bind(VkCommandBuffer command_buffer) const
{
    VkBindHeapInfoEXT resource_heap_info{ VK_STRUCTURE_TYPE_BIND_HEAP_INFO_EXT };
    resource_heap_info.heapRange.address = resource_heap_buffer.device_address;
    resource_heap_info.heapRange.size = resource_heap_buffer.size;
    resource_heap_info.reservedRangeOffset = resource_reserved_region_offset;
    resource_heap_info.reservedRangeSize = properties.minResourceHeapReservedRange;
    vkCmdBindResourceHeapEXT(command_buffer, &resource_heap_info);

    VkBindHeapInfoEXT sampler_heap_info{ VK_STRUCTURE_TYPE_BIND_HEAP_INFO_EXT };
    sampler_heap_info.heapRange.address = sampler_heap_buffer.device_address;
    sampler_heap_info.heapRange.size = sampler_heap_buffer.size;
    sampler_heap_info.reservedRangeOffset = sampler_reserved_region_offset;
    sampler_heap_info.reservedRangeSize = properties.minSamplerHeapReservedRange;
    vkCmdBindSamplerHeapEXT(command_buffer, &sampler_heap_info);
}

uint32_t Descriptor_Heap::allocate_buffer_descriptor(uint32_t count)
{
    const uint32_t descriptor_offset = round_up(current_resource_heap_offset, (uint32_t)properties.bufferDescriptorAlignment);
    current_resource_heap_offset = descriptor_offset + (uint32_t)properties.bufferDescriptorSize * count;
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

void Descriptor_Heap::write_image_descriptor(VkImage image, VkFormat image_format,
    VkDescriptorType descriptor_type, uint32_t heap_offset)
{
    VkImageViewCreateInfo image_view_ci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    image_view_ci.image = image;
    image_view_ci.viewType = VK_IMAGE_VIEW_TYPE_2D;
    image_view_ci.format = image_format;
    image_view_ci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, VK_REMAINING_MIP_LEVELS, 0, VK_REMAINING_ARRAY_LAYERS };

    VkImageDescriptorInfoEXT image_descriptor_info{ VK_STRUCTURE_TYPE_IMAGE_DESCRIPTOR_INFO_EXT };
    image_descriptor_info.pView = &image_view_ci;
    image_descriptor_info.layout = VK_IMAGE_LAYOUT_GENERAL;

    VkResourceDescriptorInfoEXT descriptor_info{ VK_STRUCTURE_TYPE_RESOURCE_DESCRIPTOR_INFO_EXT };
    descriptor_info.type = descriptor_type;
    descriptor_info.data.pImage = &image_descriptor_info;

    uint8_t* descriptor_data = static_cast<uint8_t*>(resource_heap_buffer.mapped_ptr);
    VkHostAddressRangeEXT host_range = {};
    host_range.address = descriptor_data + heap_offset;
    host_range.size = properties.imageDescriptorSize;

    VK_CHECK(vkWriteResourceDescriptorsEXT(vk.device, 1, &descriptor_info, &host_range));
}

void Descriptor_Heap::write_buffer_descriptor(VkDeviceAddressRangeEXT address_range,
    VkDescriptorType descriptor_type, uint32_t heap_offset)
{
    VkResourceDescriptorInfoEXT descriptor_info{ VK_STRUCTURE_TYPE_RESOURCE_DESCRIPTOR_INFO_EXT };
    descriptor_info.type = descriptor_type;
    descriptor_info.data.pAddressRange = address_range.address ? &address_range : nullptr;

    uint8_t* descriptor_data = static_cast<uint8_t*>(resource_heap_buffer.mapped_ptr);
    VkHostAddressRangeEXT host_range = {};
    host_range.address = descriptor_data + heap_offset;
    host_range.size = properties.bufferDescriptorSize;

    VK_CHECK(vkWriteResourceDescriptorsEXT(vk.device, 1, &descriptor_info, &host_range));
}

void Descriptor_Heap::write_acceleration_structure_descriptor(VkDeviceAddress device_address, uint32_t heap_offset)
{
    VkDeviceAddressRangeEXT address_range{
        device_address,
        0 // size is not requied for acceleration structure descriptors
    };
    VkResourceDescriptorInfoEXT descriptor_info{ VK_STRUCTURE_TYPE_RESOURCE_DESCRIPTOR_INFO_EXT };
    descriptor_info.type = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
    descriptor_info.data.pAddressRange = device_address ? &address_range : nullptr;

    uint8_t* descriptor_data = static_cast<uint8_t*>(resource_heap_buffer.mapped_ptr);
    VkHostAddressRangeEXT host_range = {};
    host_range.address = descriptor_data + heap_offset;
    host_range.size = properties.bufferDescriptorSize;

    VK_CHECK(vkWriteResourceDescriptorsEXT(vk.device, 1, &descriptor_info, &host_range));
}

void Descriptor_Heap::write_sampler_descriptor(const VkSamplerCreateInfo& sampler_ci, uint32_t heap_offset)
{
    uint8_t* descriptor_data = static_cast<uint8_t*>(sampler_heap_buffer.mapped_ptr);
    VkHostAddressRangeEXT host_range{};
    host_range.address = descriptor_data + heap_offset;
    host_range.size = properties.samplerDescriptorSize;

    VK_CHECK(vkWriteSamplerDescriptorsEXT(vk.device, 1, &sampler_ci, &host_range));
}
