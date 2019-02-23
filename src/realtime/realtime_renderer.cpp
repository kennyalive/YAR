#include "lib/common.h"
#include "realtime_renderer.h"
#include "vk.h"
#include "utils.h"

#include "io/test_scenes.h"
#include "lib/matrix.h"
#include "lib/mesh.h"
#include "reference/reference_renderer.h"

#include "glfw/glfw3.h"
#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "imgui/impl/imgui_impl_vulkan.h"
#include "imgui/impl/imgui_impl_glfw.h"

#include <cinttypes>
#include <chrono>

void Realtime_Renderer::initialize(Vk_Create_Info vk_create_info, GLFWwindow* window) {
    vk_initialize(window, vk_create_info);

    // Device properties.
    {
        VkPhysicalDeviceProperties2 physical_device_properties { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
        if (vk.raytracing_supported) {
            rt.properties = VkPhysicalDeviceRayTracingPropertiesNV { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PROPERTIES_NV };
            physical_device_properties.pNext = &rt.properties;
        }
        vkGetPhysicalDeviceProperties2(vk.physical_device, &physical_device_properties);

        printf("Device: %s\n", physical_device_properties.properties.deviceName);
        printf("Vulkan API version: %d.%d.%d\n",
            VK_VERSION_MAJOR(physical_device_properties.properties.apiVersion),
            VK_VERSION_MINOR(physical_device_properties.properties.apiVersion),
            VK_VERSION_PATCH(physical_device_properties.properties.apiVersion)
        );

        if (vk.raytracing_supported) {
            printf("\n");
            printf("VkPhysicalDeviceRayTracingPropertiesNV:\n");
            printf("  shaderGroupHandleSize = %u\n", rt.properties.shaderGroupHandleSize);
            printf("  maxRecursionDepth = %u\n", rt.properties.maxRecursionDepth);
            printf("  maxShaderGroupStride = %u\n", rt.properties.maxShaderGroupStride);
            printf("  shaderGroupBaseAlignment = %u\n", rt.properties.shaderGroupBaseAlignment);
            printf("  maxGeometryCount = %" PRIu64 "\n", rt.properties.maxGeometryCount);
            printf("  maxInstanceCount = %" PRIu64 "\n", rt.properties.maxInstanceCount);
            printf("  maxTriangleCount = %" PRIu64 "\n", rt.properties.maxTriangleCount);
            printf("  maxDescriptorSetAccelerationStructures = %u\n", rt.properties.maxDescriptorSetAccelerationStructures);
        }

    }

    // Geometry buffers.
    {
        //scene_data = load_bunny_scene(); // 144K
        scene_data = load_conference_scene(); // 330K
        //scene_data = load_buddha_scene(); // 1M
        //scene_data = load_hairball_scene(); // 2.9M

        Matrix3x4 camera_transform;
        camera_transform.set_column(0, Vector3(-1, 0, 0));
        camera_transform.set_column(1, Vector3(0, -1, 0));
        camera_transform.set_column(2, Vector3(0, 0, 1));
        camera_transform.set_column(3, Vector3(0, 3, 1));
        flying_camera.initialize(camera_transform);

        gpu_meshes.resize(scene_data.meshes.size());

        for (size_t i = 0; i < scene_data.meshes.size(); i++) {
            const Mesh_Data& mesh_data = scene_data.meshes[i];
            GPU_Mesh& mesh  = gpu_meshes[i];

            mesh.model_vertex_count = static_cast<uint32_t>(mesh_data.vertices.size());
            mesh.model_index_count = static_cast<uint32_t>(mesh_data.indices.size());

            const VkDeviceSize vertex_buffer_size = mesh_data.vertices.size() * sizeof(mesh_data.vertices[0]);
            mesh.vertex_buffer = vk_create_buffer(vertex_buffer_size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, mesh_data.vertices.data(), "vertex_buffer");

            const VkDeviceSize index_buffer_size = mesh_data.indices.size() * sizeof(mesh_data.indices[0]);
            mesh.index_buffer = vk_create_buffer(index_buffer_size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, mesh_data.indices.data(), "index_buffer");

            ASSERT(scene_data.materials[i].material_format == Material_Format::obj_material);
            mesh.material.k_diffuse = scene_data.materials[i].obj_material.k_diffuse;
            mesh.material.pad0 = 0;
            mesh.material.k_specular = scene_data.materials[i].obj_material.k_specular;
            mesh.material.pad1 = 0;
        }
    }

    // UI render pass.
    {
        VkAttachmentDescription attachments[1] = {};
        attachments[0].format           = VK_FORMAT_R16G16B16A16_SFLOAT;
        attachments[0].samples          = VK_SAMPLE_COUNT_1_BIT;
        attachments[0].loadOp           = VK_ATTACHMENT_LOAD_OP_LOAD;
        attachments[0].storeOp          = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[0].stencilLoadOp    = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[0].stencilStoreOp   = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[0].initialLayout    = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        attachments[0].finalLayout      = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentReference color_attachment_ref;
        color_attachment_ref.attachment = 0;
        color_attachment_ref.layout     = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint       = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount    = 1;
        subpass.pColorAttachments       = &color_attachment_ref;

        VkRenderPassCreateInfo create_info{ VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
        create_info.attachmentCount = (uint32_t)std::size(attachments);
        create_info.pAttachments = attachments;
        create_info.subpassCount = 1;
        create_info.pSubpasses = &subpass;

        VK_CHECK(vkCreateRenderPass(vk.device, &create_info, nullptr, &ui_render_pass));
        vk_set_debug_name(ui_render_pass, "ui_render_pass");
    }

    raster.create();
    raster.update_point_lights(scene_data.rgb_point_lights.data(), (int)scene_data.rgb_point_lights.size());

    if (vk.raytracing_supported) {
        rt.create(scene_data, gpu_meshes);
        rt.update_point_lights(scene_data.rgb_point_lights.data(), (int)scene_data.rgb_point_lights.size());
    }

    copy_to_swapchain.create();
    restore_resolution_dependent_resources();

    // ImGui setup.
    {
        ImGui::CreateContext();
        ImGui_ImplGlfw_InitForVulkan(window, true);

        ImGui_ImplVulkan_InitInfo init_info{};
        init_info.Instance          = vk.instance;
        init_info.PhysicalDevice    = vk.physical_device;
        init_info.Device            = vk.device;
        init_info.QueueFamily       = vk.queue_family_index;
        init_info.Queue             = vk.queue;
        init_info.DescriptorPool    = vk.descriptor_pool;

        ImGui_ImplVulkan_Init(&init_info, ui_render_pass);
        ImGui::StyleColorsDark();

        vk_execute(vk.command_pool, vk.queue, [](VkCommandBuffer cb) {
            ImGui_ImplVulkan_CreateFontsTexture(cb);
        });
        ImGui_ImplVulkan_InvalidateFontUploadObjects();
    }

    gpu_times.frame = time_keeper.allocate_time_interval();
    gpu_times.draw = time_keeper.allocate_time_interval();
    gpu_times.ui = time_keeper.allocate_time_interval();
    gpu_times.compute_copy = time_keeper.allocate_time_interval();
    time_keeper.initialize_time_intervals();
}

void Realtime_Renderer::shutdown() {
    VK_CHECK(vkDeviceWaitIdle(vk.device));

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    for (GPU_Mesh& mesh : gpu_meshes) {
        mesh.vertex_buffer.destroy();
        mesh.index_buffer.destroy();
    }
    gpu_meshes.clear();

    copy_to_swapchain.destroy();
    vkDestroyRenderPass(vk.device, ui_render_pass, nullptr);
    release_resolution_dependent_resources();

    raster.destroy();
    if (vk.raytracing_supported)
        rt.destroy();
    
    vk_shutdown();
}

void Realtime_Renderer::release_resolution_dependent_resources() {
    vkDestroyFramebuffer(vk.device, ui_framebuffer, nullptr);
    ui_framebuffer = VK_NULL_HANDLE;

    raster.destroy_framebuffer();
    output_image.destroy();
}

void Realtime_Renderer::restore_resolution_dependent_resources() {
    // output image
    {
        output_image = vk_create_image(vk.surface_size.width, vk.surface_size.height, VK_FORMAT_R16G16B16A16_SFLOAT,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, "output_image");

        if (raytracing) {
            vk_execute(vk.command_pool, vk.queue, [this](VkCommandBuffer command_buffer) {
                vk_cmd_image_barrier(command_buffer, output_image.handle,
                    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,  VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                    0,                                  0,
                    VK_IMAGE_LAYOUT_UNDEFINED,          VK_IMAGE_LAYOUT_GENERAL);
            });
        }
    }

    // imgui framebuffer
    {
        VkFramebufferCreateInfo create_info { VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        create_info.renderPass      = ui_render_pass;
        create_info.attachmentCount = 1;
        create_info.pAttachments    = &output_image.view;
        create_info.width           = vk.surface_size.width;
        create_info.height          = vk.surface_size.height;
        create_info.layers          = 1;

        VK_CHECK(vkCreateFramebuffer(vk.device, &create_info, nullptr, &ui_framebuffer));
    }

    raster.create_framebuffer(output_image.view);

    if (vk.raytracing_supported)
        rt.update_output_image_descriptor(output_image.view);

    copy_to_swapchain.update_resolution_dependent_descriptors(output_image.view);
}

static double last_frame_time;

void Realtime_Renderer::run_frame() {
    bool old_raytracing = raytracing;
    do_imgui();

    if (last_frame_time == 0.0) { // initialize
        last_frame_time = glfwGetTime();
    }
    double current_time = glfwGetTime();
    double dt = current_time - last_frame_time;
    last_frame_time = current_time;

    flying_camera.update(dt);

    raster.update(flying_camera.get_view_transform());

    if (vk.raytracing_supported)
        rt.update_camera_transform(flying_camera.get_camera_pose());

    draw_frame();
}

void Realtime_Renderer::draw_frame() {
    vk_begin_frame();
    time_keeper.next_frame();
    gpu_times.frame->begin();

    if (raytracing && ui_result.raytracing_toggled) {
        vk_cmd_image_barrier(vk.command_buffer, output_image.handle,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            0, VK_ACCESS_SHADER_WRITE_BIT,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);
    }

    if (raytracing)
        draw_raytraced_image();
    else
        draw_rasterized_image();

    draw_imgui();
    copy_output_image_to_swapchain();
    gpu_times.frame->end();
    vk_end_frame();
}

void Realtime_Renderer::draw_rasterized_image() {
    GPU_TIME_SCOPE(gpu_times.draw);

    VkViewport viewport{};
    viewport.width = static_cast<float>(vk.surface_size.width);
    viewport.height = static_cast<float>(vk.surface_size.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor{};
    scissor.extent = vk.surface_size;

    vkCmdSetViewport(vk.command_buffer, 0, 1, &viewport);
    vkCmdSetScissor(vk.command_buffer, 0, 1, &scissor);

    VkClearValue clear_values[2];
    clear_values[0].color = {0, 0, 0, 0.0f};
    clear_values[1].depthStencil.depth = 1.0;
    clear_values[1].depthStencil.stencil = 0;

    VkRenderPassBeginInfo render_pass_begin_info { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    render_pass_begin_info.renderPass        = raster.render_pass;
    render_pass_begin_info.framebuffer       = raster.framebuffer;
    render_pass_begin_info.renderArea.extent = vk.surface_size;
    render_pass_begin_info.clearValueCount   = (uint32_t)std::size(clear_values);
    render_pass_begin_info.pClearValues      = clear_values;

    vkCmdBeginRenderPass(vk.command_buffer, &render_pass_begin_info, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindDescriptorSets(vk.command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, raster.pipeline_layout, 0, 1, &raster.descriptor_set, 0, nullptr);
    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, raster.pipeline);

    const VkDeviceSize zero_offset = 0;
    for (const GPU_Mesh& mesh : gpu_meshes) {
        vkCmdBindVertexBuffers(vk.command_buffer, 0, 1, &mesh.vertex_buffer.handle, &zero_offset);
        vkCmdBindIndexBuffer(vk.command_buffer, mesh.index_buffer.handle, 0, VK_INDEX_TYPE_UINT32);
        vkCmdPushConstants(vk.command_buffer, raster.pipeline_layout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(GPU_Mesh_Material), &mesh.material);
        vkCmdDrawIndexed(vk.command_buffer, mesh.model_index_count, 1, 0, 0, 0);
    }

    vkCmdEndRenderPass(vk.command_buffer);
}

void Realtime_Renderer::draw_raytraced_image() {
    GPU_TIME_SCOPE(gpu_times.draw);

    vkCmdBindDescriptorSets(vk.command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_NV, rt.pipeline_layout, 0, 1, &rt.descriptor_set, 0, nullptr);
    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_NV, rt.pipeline);

    uint32_t push_constants[1] = { spp4 };
    vkCmdPushConstants(vk.command_buffer, rt.pipeline_layout, VK_SHADER_STAGE_RAYGEN_BIT_NV, 0, 4, &push_constants[0]);

    const VkBuffer sbt = rt.shader_binding_table.handle;
    const uint32_t sbt_slot_size = rt.properties.shaderGroupHandleSize;
    const uint32_t miss_offset = round_up(sbt_slot_size /* raygen slot*/, rt.properties.shaderGroupBaseAlignment);
    const uint32_t hit_offset = round_up(miss_offset + sbt_slot_size /* miss slot */, rt.properties.shaderGroupBaseAlignment);

    vkCmdTraceRaysNV(vk.command_buffer,
        sbt, 0, // raygen shader
        sbt, miss_offset, sbt_slot_size, // miss shader
        sbt, hit_offset, sbt_slot_size, // chit shader
        VK_NULL_HANDLE, 0, 0,
        vk.surface_size.width, vk.surface_size.height, 1);
}

void Realtime_Renderer::draw_imgui() {
    GPU_TIME_SCOPE(gpu_times.ui);

    ImGui::Render();

    if (raytracing) {
        vk_cmd_image_barrier(vk.command_buffer, output_image.handle,
            VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_NV,   VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_SHADER_WRITE_BIT,             VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
            VK_IMAGE_LAYOUT_GENERAL,                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    } else {
        vk_cmd_image_barrier(vk.command_buffer, output_image.handle,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,          VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            0,                                          VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,   VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    }

    VkRenderPassBeginInfo render_pass_begin_info{ VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    render_pass_begin_info.renderPass           = ui_render_pass;
    render_pass_begin_info.framebuffer          = ui_framebuffer;
    render_pass_begin_info.renderArea.extent    = vk.surface_size;

    vkCmdBeginRenderPass(vk.command_buffer, &render_pass_begin_info, VK_SUBPASS_CONTENTS_INLINE);
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), vk.command_buffer);
    vkCmdEndRenderPass(vk.command_buffer);

    if (raytracing) {
        vk_cmd_image_barrier(vk.command_buffer, output_image.handle,
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,  VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,           VK_ACCESS_SHADER_READ_BIT,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,       VK_IMAGE_LAYOUT_GENERAL);
    } else {
        vk_cmd_image_barrier(vk.command_buffer, output_image.handle,
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,  VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,           VK_ACCESS_SHADER_READ_BIT,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }
}

void Realtime_Renderer::copy_output_image_to_swapchain() {
    GPU_TIME_SCOPE(gpu_times.compute_copy);

    const uint32_t group_size_x = 32; // according to shader
    const uint32_t group_size_y = 32;

    uint32_t group_count_x = (vk.surface_size.width + group_size_x - 1) / group_size_x;
    uint32_t group_count_y = (vk.surface_size.height + group_size_y - 1) / group_size_y;

    if (raytracing) {
        vk_cmd_image_barrier(vk.command_buffer, output_image.handle,
            VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_NV,   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_SHADER_WRITE_BIT,             VK_ACCESS_SHADER_READ_BIT,
            VK_IMAGE_LAYOUT_GENERAL,                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,  VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0,                                  VK_ACCESS_SHADER_WRITE_BIT,
        VK_IMAGE_LAYOUT_UNDEFINED,          VK_IMAGE_LAYOUT_GENERAL);

    uint32_t push_constants[] = { vk.surface_size.width, vk.surface_size.height };

    vkCmdPushConstants(vk.command_buffer, copy_to_swapchain.pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT,
        0, sizeof(push_constants), push_constants);

    vkCmdBindDescriptorSets(vk.command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, copy_to_swapchain.pipeline_layout,
        0, 1, &copy_to_swapchain.sets[vk.swapchain_image_index], 0, nullptr);

    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, copy_to_swapchain.pipeline);
    vkCmdDispatch(vk.command_buffer, group_count_x, group_count_y, 1);

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,   VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
        VK_ACCESS_SHADER_WRITE_BIT,             0,
        VK_IMAGE_LAYOUT_GENERAL,                VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);

    if (raytracing) {
        vk_cmd_image_barrier(vk.command_buffer, output_image.handle,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,   VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_NV,
            VK_ACCESS_SHADER_READ_BIT,              VK_ACCESS_SHADER_WRITE_BIT,
            VK_IMAGE_LAYOUT_UNDEFINED,              VK_IMAGE_LAYOUT_GENERAL);
    }
}

void Realtime_Renderer::do_imgui() {
    ui_result = UI_Result{};
    ImGuiIO& io = ImGui::GetIO();

    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (show_ui) {
        const float DISTANCE = 10.0f;
        static int corner = 0;

        ImVec2 window_pos = ImVec2((corner & 1) ? ImGui::GetIO().DisplaySize.x - DISTANCE : DISTANCE,
                                   (corner & 2) ? ImGui::GetIO().DisplaySize.y - DISTANCE : DISTANCE);

        ImVec2 window_pos_pivot = ImVec2((corner & 1) ? 1.0f : 0.0f, (corner & 2) ? 1.0f : 0.0f);

        if (corner != -1)
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
        ImGui::SetNextWindowBgAlpha(0.3f);

        if (ImGui::Begin("UI", &show_ui, 
            (corner != -1 ? ImGuiWindowFlags_NoMove : 0) | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav))
        {
            ImGui::Text("%.1f FPS (%.3f ms/frame)", ImGui::GetIO().Framerate, 1000.0f / ImGui::GetIO().Framerate);
            ImGui::Text("Frame time         : %.2f ms", gpu_times.frame->length_ms);
            ImGui::Text("Draw time          : %.2f ms", gpu_times.draw->length_ms);
            ImGui::Text("UI time            : %.2f ms", gpu_times.ui->length_ms);
            ImGui::Text("Compute copy time  : %.2f ms", gpu_times.compute_copy->length_ms);
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::Checkbox("Vertical sync", &vsync);

            if (!vk.raytracing_supported) {
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
            }
            ui_result.raytracing_toggled = ImGui::Checkbox("Raytracing", &raytracing);
            ImGui::Checkbox("4 rays per pixel", &spp4);
            if (!vk.raytracing_supported) {
                ImGui::PopItemFlag();
                ImGui::PopStyleVar();
            }

            bool disable_button = reference_render_active;
            if (disable_button)
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);

            if (ImGui::Button("Render reference image"))
            {
                reference_render_active = true;

                Render_Reference_Image_Params params {};
                params.image_resolution = Vector2i{ (int)vk.surface_size.width, (int)vk.surface_size.height };

                params.render_region.p0 = Vector2i{};
                params.render_region.p1 = params.image_resolution;

                params.scene_data = &scene_data;
                params.camera_to_world_vk = flying_camera.get_camera_pose();

                /*params.crop_x = 462;
                params.crop_y = 302;
                params.crop_w = 3;
                params.crop_h = 3;*/
                     
                reference_render_thread = std::thread(render_reference_image, params, &reference_render_active);
            }

            if (disable_button)
                ImGui::PopItemFlag();

            if (!reference_render_active && reference_render_thread.joinable())
                reference_render_thread.join();

            if (ImGui::BeginPopupContextWindow()) {
                if (ImGui::MenuItem("Custom",       NULL, corner == -1)) corner = -1;
                if (ImGui::MenuItem("Top-left",     NULL, corner == 0)) corner = 0;
                if (ImGui::MenuItem("Top-right",    NULL, corner == 1)) corner = 1;
                if (ImGui::MenuItem("Bottom-left",  NULL, corner == 2)) corner = 2;
                if (ImGui::MenuItem("Bottom-right", NULL, corner == 3)) corner = 3;
                if (ImGui::MenuItem("Close")) show_ui = false;
                ImGui::EndPopup();
            }
        }
        ImGui::End();
    }
}
