#include "std.h"
#include "lib/common.h"
#include "descriptor_offsets.h"

#include "descriptor_heap.h"

#include "shaders/shared.slang"

static_assert(static_cast<uint32_t>(Image_Index::none) == None_Texture_Index);
static_assert(static_cast<uint32_t>(Image_Index::first_project_image) == First_Project_Image);

void Descriptor_Offsets::initialize(Descriptor_Heap& descriptor_heap)
{
    image_sampler = descriptor_heap.allocate_sampler_descriptor();
    scene_info_buffer = descriptor_heap.allocate_buffer_descriptor();
    instance_infos = descriptor_heap.allocate_buffer_descriptor();
    mesh_infos = descriptor_heap.allocate_buffer_descriptor();
    mesh_vertex_data = descriptor_heap.allocate_buffer_descriptor();
    mesh_index_data = descriptor_heap.allocate_buffer_descriptor();
    accelerator = descriptor_heap.allocate_buffer_descriptor();
    lambertian_materials = descriptor_heap.allocate_buffer_descriptor();
    point_lights = descriptor_heap.allocate_buffer_descriptor();
    directional_lights = descriptor_heap.allocate_buffer_descriptor();
    rect_lights = descriptor_heap.allocate_buffer_descriptor();

    // At launch we don't know the number of images (and it changes per project),
    // but the offsets of the image descriptors are the same, so request a single
    // descriptor to get the offset. We validate that there is enough heap space
    // when loading the project.
    images = descriptor_heap.allocate_image_descriptor(1);
}

uint32_t Descriptor_Offsets::get_image_descriptor_offset(Image_Index image_index) const
{
    return images + static_cast<uint32_t>(image_index) * vk_image_descriptor_size();
}

std::vector<VkDescriptorSetAndBindingMappingEXT> Descriptor_Offsets::get_descriptor_mappings() const
{
    const VkSpirvResourceTypeFlagBitsEXT resource_type_read_only_storage = VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT;
    const VkSpirvResourceTypeFlagBitsEXT resource_type_read_write_storage = VK_SPIRV_RESOURCE_TYPE_READ_WRITE_STORAGE_BUFFER_BIT_EXT;
    const VkSpirvResourceTypeFlagBitsEXT resource_type_accelerator = VK_SPIRV_RESOURCE_TYPE_ACCELERATION_STRUCTURE_BIT_EXT;

    std::vector<VkDescriptorSetAndBindingMappingEXT> mappings;

    // Images
    mappings.push_back(map_binding_to_heap_offset(
        GLOBAL_SET, GLOBAL_BINDING_SAMPLED_IMAGES, VK_SPIRV_RESOURCE_TYPE_SAMPLED_IMAGE_BIT_EXT,
        images, vk_image_descriptor_size()
    ));
    mappings.push_back(map_binding_to_heap_offset(
        GLOBAL_SET, GLOBAL_BINDING_STORAGE_IMAGES_RGBA_F16, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT,
        images, vk_image_descriptor_size()
    ));
    mappings.push_back(map_binding_to_heap_offset(
        GLOBAL_SET, GLOBAL_BINDING_STORAGE_IMAGES_RGBA8, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT,
        images, vk_image_descriptor_size()
    ));
    mappings.push_back(map_binding_to_heap_offset(
        GLOBAL_SET, GLOBAL_BINDING_SAMPLER, VK_SPIRV_RESOURCE_TYPE_SAMPLER_BIT_EXT, image_sampler
    ));

    // Scene base resources
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_BASE_SET, SCENE_BASE_BINDING_INSTANCE_INFO, resource_type_read_only_storage, instance_infos
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_BASE_SET, SCENE_BASE_BINDING_MESH_INFO, resource_type_read_only_storage, mesh_infos
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_BASE_SET, SCENE_BASE_BINDING_MESH_VERTEX_DATA, resource_type_read_only_storage, mesh_vertex_data
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_BASE_SET, SCENE_BASE_BINDING_MESH_INDEX_DATA, resource_type_read_only_storage, mesh_index_data
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_BASE_SET, SCENE_BASE_BINDING_SCENE_INFO, resource_type_read_only_storage, scene_info_buffer
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_BASE_SET, SCENE_BASE_BINDING_ACCELERATOR, resource_type_accelerator, accelerator
    ));

    // Materials
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_MATERIAL_SET, SCENE_MATERIAL_BINDING_LAMBERTIAN, resource_type_read_write_storage, lambertian_materials
    ));

    // Lights
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_LIGHT_SET, SCENE_LIGHT_BINDING_POINT, resource_type_read_only_storage, point_lights
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_LIGHT_SET, SCENE_LIGHT_BINDING_DIRECTIONAL, resource_type_read_only_storage, directional_lights
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_LIGHT_SET, SCENE_LIGHT_BINDING_RECT, resource_type_read_only_storage, rect_lights
    ));
    return mappings;
}
