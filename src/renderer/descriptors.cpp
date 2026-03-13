#include "std.h"
#include "lib/common.h"
#include "descriptors.h"

#include "descriptor_heap.h"

void Descriptors::initialize(Descriptor_Heap& descriptor_heap)
{
    output_image = descriptor_heap.allocate_image_descriptor();
    swapchain_images = descriptor_heap.allocate_image_descriptor(max_swapchain_images);
    image_sampler = descriptor_heap.allocate_sampler_descriptor();

    lambertian_materials = descriptor_heap.allocate_buffer_descriptor();
    point_lights = descriptor_heap.allocate_buffer_descriptor();
    directional_lights = descriptor_heap.allocate_buffer_descriptor();
    diffuse_rectangular_lights = descriptor_heap.allocate_buffer_descriptor();

    image_descriptor_size = (uint32_t)descriptor_heap.properties.imageDescriptorSize;
    buffer_descriptor_size = (uint32_t)descriptor_heap.properties.bufferDescriptorSize;
    sampler_descriptor_size = (uint32_t)descriptor_heap.properties.samplerDescriptorSize;
}
