#include "std.h"
#include "lib/common.h"
#include "kernels.h"

#include "gpu_scene.h"

void Kernels::create_global_kernels(Descriptor_Offsets& descriptor_offsets)
{
}

void Kernels::destroy_global_kernels()
{
}

void Kernels::create_scene_kernels(
    Descriptor_Offsets& descriptor_offsets,
    Descriptor_Heap& descriptor_heap,
    const GPU_Scene& gpu_scene,
    const Scene& scene)
{
    patch_materials.create(descriptor_offsets);
}

void Kernels::destroy_scene_kernels()
{
    patch_materials.destroy();
}
