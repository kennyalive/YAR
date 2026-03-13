#include "std.h"
#include "lib/common.h"
#include "descriptors.h"

#include "descriptor_heap.h"

void Descriptors::initialize(Descriptor_Heap& descriptor_heap)
{
    output_image = descriptor_heap.allocate_image_descriptor();
    swapchain_images = descriptor_heap.allocate_image_descriptor(max_swapchain_images);

    image_descriptor_size = (uint32_t)descriptor_heap.properties.imageDescriptorSize;
}
