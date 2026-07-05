#include "std.h"
#include "lib/common.h"
#include "kernels.h"

#include "gpu_scene.h"

void Kernels::create_global_kernels(Global_Descriptors& global_descriptors)
{
}

void Kernels::destroy_global_kernels()
{
}

void Kernels::create_scene_kernels(
    Global_Descriptors& global_descriptors, 
    Descriptor_Heap& descriptor_heap,
    const GPU_Scene& gpu_scene,
    const Scene& scene)
{
    patch_materials.create(gpu_scene.descriptors);
}

void Kernels::destroy_scene_kernels()
{
    patch_materials.destroy();
}
