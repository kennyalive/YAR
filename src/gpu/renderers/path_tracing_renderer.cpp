#include "std.h"
#include "lib/common.h"
#include "path_tracing_renderer.h"
#include "../descriptor_heap.h"
#include "../descriptor_offsets.h"
#include "../gpu_scene.h"

constexpr VkFormat output_image_format = VK_FORMAT_R16G16B16A16_SFLOAT;

void Path_Tracing_Renderer::initialize(
    const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings,
    Vk_Time_Keeper& time_keeper)
{
    apply_tone_mapping.create(descriptor_mappings);
    timer_draw = time_keeper.allocate_timer("path_draw");
    timer_tonemap = time_keeper.allocate_timer("path_tone_map");
    timer_compute_copy = time_keeper.allocate_timer("path_compute_copy");
}

void Path_Tracing_Renderer::destroy()
{
    apply_tone_mapping.destroy();
    if (path_tracing.pipeline != VK_NULL_HANDLE) {
        path_tracing.destroy();
    }
}

void Path_Tracing_Renderer::create_scene_kernels(const Descriptor_Offsets& descriptor_offsets, const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings)
{
    const uint32_t output_image_offset = descriptor_offsets.get_image_descriptor_offset(Image_Index::path_tracer_output);
    path_tracing.create(output_image_offset, descriptor_mappings);
}

void Path_Tracing_Renderer::create_resolution_dependent_resources(Descriptor_Heap& descriptor_heap,
    uint32_t output_image_heap_offset, uint32_t tonemap_image_heap_offset, uint32_t swapchain_images_heap_offset)
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

        descriptor_heap.write_image_descriptor(output_image.handle, output_image.format,
            VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, output_image_heap_offset);
    }
    // tone mapped image
    {
        tonemapped_image = vk_create_image(vk.surface_size.width, vk.surface_size.height, output_image_format,
            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, "tonemapped_image");

        vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
            vk_cmd_image_barrier(command_buffer, tonemapped_image.handle,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_GENERAL);
            });

        descriptor_heap.write_image_descriptor(tonemapped_image.handle, tonemapped_image.format,
            VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, tonemap_image_heap_offset);
    }
    copy_to_swapchain.create(tonemap_image_heap_offset, swapchain_images_heap_offset);
}

void Path_Tracing_Renderer::destroy_resolution_dependent_resources()
{
    output_image.destroy();
    tonemapped_image.destroy();
    copy_to_swapchain.destroy();
}

void Path_Tracing_Renderer::render(const GPU_Scene& gpu_scene)
{
    if (gpu_scene.loaded) {
        VK_TIME_SCOPE(timer_draw);
        path_tracing.dispatch();
    }

    {
        VK_TIME_SCOPE(timer_tonemap);
        apply_tone_mapping.dispatch((uint32_t)Image_Index::path_tracer_output, (uint32_t)Image_Index::path_tracer_tonemap);
    }

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL);

    {
        VK_TIME_SCOPE(timer_compute_copy);
        copy_to_swapchain.dispatch();
    }
}
