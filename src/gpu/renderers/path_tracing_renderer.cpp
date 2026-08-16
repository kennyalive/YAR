#include "std.h"
#include "lib/common.h"
#include "path_tracing_renderer.h"
#include "../descriptor_heap.h"
#include "../descriptor_heap_layout.h"
#include "../gpu_scene.h"
#include "../kernels.h"

constexpr VkFormat output_image_format = VK_FORMAT_R16G16B16A16_SFLOAT;

void Path_Tracing_Renderer::initialize(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings, Vk_Time_Keeper& time_keeper)
{
    path_tracing.create(descriptor_mappings);
    timer_draw = time_keeper.allocate_timer("path_draw");
    timer_tonemap = time_keeper.allocate_timer("path_tone_map");
    timer_compute_copy = time_keeper.allocate_timer("path_compute_copy");
}

void Path_Tracing_Renderer::destroy()
{
    if (path_tracing.pipeline != VK_NULL_HANDLE) {
        path_tracing.destroy();
    }
}

void Path_Tracing_Renderer::activate()
{
    vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
        VkClearColorValue clear_color{};
        clear_color.float32[3] = 1.f;
        VkImageSubresourceRange range{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
        vkCmdClearColorImage(command_buffer, output_image.handle, VK_IMAGE_LAYOUT_GENERAL, &clear_color, 1, &range);
        vkCmdClearColorImage(command_buffer, tonemap.handle, VK_IMAGE_LAYOUT_GENERAL, &clear_color, 1, &range);
    });
    accumulation_index = 0;
}

void Path_Tracing_Renderer::on_camera_changed()
{
    accumulation_index = 0;
}

void Path_Tracing_Renderer::create_resolution_dependent_resources()
{
    // output image
    {
        output_image = vk_create_image(vk.surface_size.width, vk.surface_size.height, output_image_format,
            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, "output_image");

        vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
            vk_cmd_image_barrier(command_buffer, output_image.handle,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_GENERAL);
            });
    }
    // tone mapped image
    {
        tonemap = vk_create_image(vk.surface_size.width, vk.surface_size.height, output_image_format,
            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, "tonemapped_image");

        vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
            vk_cmd_image_barrier(command_buffer, tonemap.handle,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_GENERAL);
            });
    }
}

void Path_Tracing_Renderer::destroy_resolution_dependent_resources()
{
    output_image.destroy();
    tonemap.destroy();
}

void Path_Tracing_Renderer::write_resolution_dependent_descriptors(const Descriptor_Heap& descriptor_heap, const Descriptor_Heap_Layout& layout)
{
    const uint32_t output_image_offset = layout.get_image_descriptor_offset(Image_Descriptor_Index::path_tracer_output);
    descriptor_heap.write_image_descriptor(output_image.handle, output_image.format,
        VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, output_image_offset);

    const uint32_t tonemap_offset = layout.get_image_descriptor_offset(Image_Descriptor_Index::path_tracer_tonemap);
    descriptor_heap.write_image_descriptor(tonemap.handle, tonemap.format,
        VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, tonemap_offset);
}

void Path_Tracing_Renderer::render(const GPU_Scene& gpu_scene, const Kernels& kernels)
{
    const uint32_t output_index = uint32_t(Image_Descriptor_Index::path_tracer_output);
    const uint32_t tonemap_index = uint32_t(Image_Descriptor_Index::path_tracer_tonemap);

    {
        VK_TIME_SCOPE(timer_draw);
        path_tracing.dispatch((uint32_t)Image_Descriptor_Index::path_tracer_output, accumulation_index);
    }
    {
        VK_TIME_SCOPE(timer_tonemap);
        kernels.apply_tone_mapping.dispatch(output_index, tonemap_index);
    }
    {
        VK_TIME_SCOPE(timer_compute_copy);
        kernels.copy_to_swapchain.dispatch(tonemap_index);
    }
    accumulation_index++;
}
