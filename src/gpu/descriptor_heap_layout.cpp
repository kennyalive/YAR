#include "std.h"
#include "lib/common.h"
#include "descriptor_heap_layout.h"
#include "shaders/shared.slang"

static_assert(uint32_t(Image_Descriptor_Index::none) == None_Texture_Index);
static_assert(uint32_t(Image_Descriptor_Index::first_project_image) == First_Project_Image);

static uint32_t next_buffer_descriptor_offset(uint32_t& current_offset)
{
    const auto& props = vk.descriptor_heap_properties;
    const uint32_t descriptor_offset = round_up(current_offset, uint32_t(props.bufferDescriptorAlignment));
    current_offset = descriptor_offset + uint32_t(props.bufferDescriptorSize);
    return descriptor_offset;
}

static uint32_t next_image_descriptor_offset(uint32_t& current_offset, uint32_t count = 1)
{
    const auto& props = vk.descriptor_heap_properties;
    const uint32_t descriptor_offset = round_up(current_offset, uint32_t(props.imageDescriptorAlignment));
    current_offset = descriptor_offset + uint32_t(props.imageDescriptorSize * count);
    return descriptor_offset;
}

static uint32_t next_sampler_descriptor_offset(uint32_t& current_offset)
{
    const auto& props = vk.descriptor_heap_properties;
    const uint32_t descriptor_offset = round_up(current_offset, uint32_t(props.samplerDescriptorAlignment));
    current_offset = descriptor_offset + uint32_t(props.samplerDescriptorSize);
    return descriptor_offset;
}

void Descriptor_Heap_Layout::initialize()
{
    uint32_t resource_heap_offset = 0;

    // Initialize storage buffer descriptors offsets
    uint32_t* p_offsets[] = {
        &scene_info,
        &instance_infos,
        &mesh_infos,
        &mesh_vertex_data,
        &mesh_index_data,
        &lambertian_materials,
        &point_lights,
        &directional_lights,
        &rect_lights
    };
    storage_buffer_descriptor_offsets.reserve(std::size(p_offsets));
    for (uint32_t* p_offset : p_offsets) {
        // Write into offset variable (e.g. instance_infos)
        *p_offset = next_buffer_descriptor_offset(resource_heap_offset);
        // Add offset to the array of all storage descriptor
        storage_buffer_descriptor_offsets.push_back(*p_offset);
    }

    // Accelerator descriptor has size of a buffer descriptor but uses a different descriptor type.
    // The descriptor clear code depends on the descriptor type, so don't put accelerator into
    // storage_buffer_descriptor_offsets.
    accelerator = next_buffer_descriptor_offset(resource_heap_offset);

    // At launch we don't know the number of images (and it changes per project),
    // but the offset of the image descriptor array is always the same, so request
    // a single descriptor to get the offset. Since we don't know the number of images
    // this should be the last resource heap allocation.
    images = next_image_descriptor_offset(resource_heap_offset, 1);

    // Sampler heap
    uint32_t sampler_heap_offset = 0;
    image_sampler = next_sampler_descriptor_offset(sampler_heap_offset);
}

uint32_t Descriptor_Heap_Layout::get_image_descriptor_offset(Image_Descriptor_Index index) const
{
    return images + uint32_t(index) * vk_image_descriptor_size();
}

uint32_t Descriptor_Heap_Layout::get_total_descriptor_data_size(uint32_t project_image_count) const
{
    // Project images is the last descriptor region.
    // Its size depends depends on the number of project images.
    const uint32_t first_project_image_offset = get_image_descriptor_offset(Image_Descriptor_Index::first_project_image);
    return first_project_image_offset + project_image_count * vk_image_descriptor_size();
}

std::vector<VkDescriptorSetAndBindingMappingEXT> Descriptor_Heap_Layout::get_descriptor_mappings() const
{
    const VkSpirvResourceTypeFlagBitsEXT resource_type_read_only_storage = VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT;
    const VkSpirvResourceTypeFlagBitsEXT resource_type_read_write_storage = VK_SPIRV_RESOURCE_TYPE_READ_WRITE_STORAGE_BUFFER_BIT_EXT;
    const VkSpirvResourceTypeFlagBitsEXT resource_type_accelerator = VK_SPIRV_RESOURCE_TYPE_ACCELERATION_STRUCTURE_BIT_EXT;

    std::vector<VkDescriptorSetAndBindingMappingEXT> mappings;

    // Scene
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_BASE_SET, SCENE_BASE_BINDING_SCENE_INFO, resource_type_read_only_storage, scene_info
    ));
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

    // Samplers
    mappings.push_back(map_binding_to_heap_offset(
        GLOBAL_SET, GLOBAL_BINDING_SAMPLER, VK_SPIRV_RESOURCE_TYPE_SAMPLER_BIT_EXT, image_sampler
    ));
    return mappings;
}
