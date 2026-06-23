#pragma once

#include "renderer/acceleration_structure.h"

struct Scene;
struct Descriptor_Heap;
struct Descriptors;

struct Direct_Lighting {
    Vk_Intersection_Accelerator accelerator;
    uint32_t accelerator_heap_offset = 0;
    Vk_Buffer shader_binding_table;
    VkPipeline pipeline = VK_NULL_HANDLE;

    void create(Descriptor_Heap& descriptor_heap, const Descriptors& descriptors,
        const std::vector<VkDescriptorSetAndBindingMappingEXT>& global_heap_mappings, 
        const Scene& scene, const std::vector<GPU_Mesh>& gpu_meshes
    );
    void destroy();
    void dispatch();

private:
    void create_pipeline(const Descriptors& descriptors,
        const std::vector<VkDescriptorSetAndBindingMappingEXT>& global_heap_mappings,
        const std::vector<GPU_Mesh>& gpu_meshes
    );
};
