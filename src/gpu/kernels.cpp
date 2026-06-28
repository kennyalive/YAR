#include "std.h"
#include "lib/common.h"
#include "kernels.h"

#include "gpu_scene.h"

void Kernels::create_global_kernels(Global_Descriptors& global_descriptors)
{
    apply_tone_mapping.create(global_descriptors);
    copy_to_swapchain.create(global_descriptors);
}

void Kernels::create_scene_kernels(
    Global_Descriptors& global_descriptors, 
    Descriptor_Heap& descriptor_heap,
    const GPU_Scene& gpu_scene,
    const Scene& scene)
{
    patch_materials.create(gpu_scene.descriptors);

    const std::vector<VkDescriptorSetAndBindingMappingEXT> scene_descriptor_mappings = gpu_scene.get_scene_descriptor_mappings();
    direct_lighting.create(descriptor_heap, global_descriptors, scene_descriptor_mappings, scene, gpu_scene.meshes);
    path_tracing.create(descriptor_heap, global_descriptors, scene_descriptor_mappings, scene, gpu_scene.meshes);
}

void Kernels::destroy_global_kernels()
{
    apply_tone_mapping.destroy();
    copy_to_swapchain.destroy();
}

void Kernels::destroy_scene_kernels()
{
    patch_materials.destroy();
    direct_lighting.destroy();
    path_tracing.destroy();
}
