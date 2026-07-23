#include "std.h"
#include "lib/common.h"
#include "kernels.h"

#include "descriptor_heap_layout.h"

void Kernels::create_kernels(const Descriptor_Heap_Layout& layout)
{
    std::vector<VkDescriptorSetAndBindingMappingEXT> descriptor_mappings = layout.get_descriptor_mappings();
    patch_materials.create(descriptor_mappings);
    apply_tone_mapping.create(descriptor_mappings);
    copy_to_swapchain.create(descriptor_mappings);
}

void Kernels::destroy_kernels()
{
    patch_materials.destroy();
    apply_tone_mapping.destroy();
    copy_to_swapchain.destroy();
}
