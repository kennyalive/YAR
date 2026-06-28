#include "std.h"
#include "lib/common.h"
#include "descriptors.h"

#include "descriptor_heap.h"

constexpr uint32_t max_swapchain_images = 8;

void Global_Descriptors::initialize(Descriptor_Heap& descriptor_heap)
{
    output_image = descriptor_heap.allocate_image_descriptor();
    tonemapped_image = descriptor_heap.allocate_image_descriptor();
    swapchain_images = descriptor_heap.allocate_image_descriptor(max_swapchain_images);
}

void Scene_Descriptors::initialize(Descriptor_Heap& descriptor_heap)
{
    image_sampler = descriptor_heap.allocate_sampler_descriptor();
    lambertian_materials = descriptor_heap.allocate_buffer_descriptor();
    point_lights = descriptor_heap.allocate_buffer_descriptor();
    directional_lights = descriptor_heap.allocate_buffer_descriptor();
    rect_lights = descriptor_heap.allocate_buffer_descriptor();
}
