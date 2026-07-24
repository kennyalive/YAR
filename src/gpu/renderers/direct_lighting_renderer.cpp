#include "std.h"
#include "lib/common.h"
#include "direct_lighting_renderer.h"
#include "../descriptor_heap.h"
#include "../descriptor_heap_layout.h"
#include "../gpu_scene.h"
#include "../kernels.h"

constexpr VkFormat output_image_format = VK_FORMAT_R16G16B16A16_SFLOAT;

void Direct_Lighting_Renderer::initialize(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings, Vk_Time_Keeper& time_keeper)
{
    direct_lighting.create(descriptor_mappings);
    timer_draw = time_keeper.allocate_timer("direct_draw");
    timer_tonemap = time_keeper.allocate_timer("direct_tone_map");
    timer_compute_copy = time_keeper.allocate_timer("direct_compute_copy");
}

void Direct_Lighting_Renderer::destroy()
{
    if (direct_lighting.pipeline != VK_NULL_HANDLE) {
        direct_lighting.destroy();
    }
}

void Direct_Lighting_Renderer::create_resolution_dependent_resources()
{
    // output image
    {
        output_image = vk_create_image(vk.surface_size.width, vk.surface_size.height, output_image_format,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, "output_image");

        vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
            vk_cmd_image_barrier(command_buffer, output_image.handle,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_GENERAL);
            });
    }
    // tone mapped image
    {
        tonemap = vk_create_image(vk.surface_size.width, vk.surface_size.height, output_image_format,
            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, "tonemapped_image");

        vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
            vk_cmd_image_barrier(command_buffer, tonemap.handle,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_GENERAL);
            });
    }
}

void Direct_Lighting_Renderer::write_resolution_dependent_descriptors(const Descriptor_Heap& descriptor_heap, const Descriptor_Heap_Layout& layout)
{
    const uint32_t output_image_offset = layout.get_image_descriptor_offset(Image_Descriptor_Index::direct_lighting_output);
    descriptor_heap.write_image_descriptor(output_image.handle, output_image.format,
        VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, output_image_offset);

    const uint32_t tonemap_offset = layout.get_image_descriptor_offset(Image_Descriptor_Index::direct_lighting_tonemap);
    descriptor_heap.write_image_descriptor(tonemap.handle, tonemap.format,
        VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, tonemap_offset);
}

void Direct_Lighting_Renderer::destroy_resolution_dependent_resources()
{
    output_image.destroy();
    tonemap.destroy();
}

void Direct_Lighting_Renderer::render(const GPU_Scene& gpu_scene, const Kernels& kernels)
{
    const uint32_t output_index = uint32_t(Image_Descriptor_Index::direct_lighting_output);
    const uint32_t tonemap_index = uint32_t(Image_Descriptor_Index::direct_lighting_tonemap);

    if (gpu_scene.loaded) {
        VK_TIME_SCOPE(timer_draw);
        direct_lighting.dispatch(output_index);
    }
    {
        VK_TIME_SCOPE(timer_tonemap);
        kernels.apply_tone_mapping.dispatch(output_index, tonemap_index);
    }
    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL);
    {
        VK_TIME_SCOPE(timer_compute_copy);
        kernels.copy_to_swapchain.dispatch(tonemap_index);
    }
}
