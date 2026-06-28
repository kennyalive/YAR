#pragma once

#include "gpu/acceleration_structure.h"

struct Scene;
struct Descriptor_Heap;
struct Global_Descriptors;

struct Direct_Lighting {
    Vk_Intersection_Accelerator accelerator;
    uint32_t accelerator_heap_offset = 0;
    Vk_Buffer shader_binding_table;
    VkPipeline pipeline = VK_NULL_HANDLE;

    void create(Descriptor_Heap& descriptor_heap, const Global_Descriptors& global_descriptors,
        const std::vector<VkDescriptorSetAndBindingMappingEXT>& scene_descriptor_mappings,
        const Scene& scene, const std::vector<GPU_Mesh>& gpu_meshes
    );
    void destroy();
    void dispatch();

private:
    void create_pipeline(const Global_Descriptors& global_descriptors,
        const std::vector<VkDescriptorSetAndBindingMappingEXT>& scene_descriptor_mappings
    );
};
