#include "std.h"
#include "lib/common.h"
#include "kernels.h"

#include "descriptor_offsets.h"

void Kernels::create_kernels(const Descriptor_Offsets& descriptor_offsets)
{
    std::vector<VkDescriptorSetAndBindingMappingEXT> descriptor_mappings = descriptor_offsets.get_descriptor_mappings();
    patch_materials.create(descriptor_mappings);
    apply_tone_mapping.create(descriptor_mappings);
}

void Kernels::destroy_kernels()
{
    patch_materials.destroy();
    apply_tone_mapping.destroy();
}
