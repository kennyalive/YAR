#include "std.h"
#include "lib/common.h"
#include "yar.h"

#include "shaders/shared.slang"
#include "vk.h"

#include "lib/matrix.h"
#include "lib/scene_loader.h"
#include "ref/reference_renderer.h"
#include "ref/scene_context.h"

#include "glfw/glfw3.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_vulkan.h"
#include "imgui/imgui_impl_glfw.h"

void YAR::initialize(GLFWwindow* window, int gpu_index) {
    std::array instance_extensions = {
        VK_KHR_SURFACE_EXTENSION_NAME,
#ifdef VK_USE_PLATFORM_WIN32_KHR
        VK_KHR_WIN32_SURFACE_EXTENSION_NAME,
#endif
        VK_EXT_DEBUG_UTILS_EXTENSION_NAME
    };
    std::array device_extensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME,
        VK_EXT_ROBUSTNESS_2_EXTENSION_NAME, // nullDescriptor feature
        VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME, // imgui v1.90.6 WIP uses extension endpoints instead of core
        VK_EXT_DESCRIPTOR_HEAP_EXTENSION_NAME,
        VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
        VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME, // required by VK_KHR_acceleration_structure
        VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME,
    };
    // use non-srgb formats for swapchain images, so we can render to swapchain from compute,
    // also it means we should do srgb encoding manually.
    std::array surface_formats = { VK_FORMAT_R8G8B8A8_UNORM };

    // Specify required features.
    VkPhysicalDeviceFeatures2 features2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
    Vk_PNexer pnexer(features2);

    VkPhysicalDeviceBufferDeviceAddressFeatures buffer_device_address_features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES };
    buffer_device_address_features.bufferDeviceAddress = VK_TRUE;
    pnexer.next(buffer_device_address_features);

    VkPhysicalDeviceDescriptorHeapFeaturesEXT descriptor_heap_features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_HEAP_FEATURES_EXT };
    descriptor_heap_features.descriptorHeap = VK_TRUE;
    pnexer.next(descriptor_heap_features);

    VkPhysicalDeviceDescriptorIndexingFeatures descriptor_indexing_features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_INDEXING_FEATURES };
    descriptor_indexing_features.runtimeDescriptorArray = VK_TRUE;
    pnexer.next(descriptor_indexing_features);

    VkPhysicalDeviceSynchronization2Features synchronization2_features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SYNCHRONIZATION_2_FEATURES };
    synchronization2_features.synchronization2 = VK_TRUE;
    pnexer.next(synchronization2_features);

    VkPhysicalDeviceDynamicRenderingFeatures dynamic_rendering_features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES };
    dynamic_rendering_features.dynamicRendering = VK_TRUE;
    pnexer.next(dynamic_rendering_features);

    VkPhysicalDeviceAccelerationStructureFeaturesKHR acceleration_structure_features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR };
    acceleration_structure_features.accelerationStructure = VK_TRUE;
    pnexer.next(acceleration_structure_features);

    VkPhysicalDeviceRayTracingPipelineFeaturesKHR ray_tracing_pipeline_features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR };
    ray_tracing_pipeline_features.rayTracingPipeline = VK_TRUE;
    pnexer.next(ray_tracing_pipeline_features);

    VkPhysicalDeviceRobustness2FeaturesEXT robustness2_features{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ROBUSTNESS_2_FEATURES_EXT };
    robustness2_features.nullDescriptor = VK_TRUE;
    pnexer.next(robustness2_features);

    Vk_Init_Params vk_init_params;
    vk_init_params.error_reporter = &error;
    vk_init_params.physical_device_index = gpu_index;
    vk_init_params.vsync = ui.vsync;
    vk_init_params.instance_extensions = std::span{ instance_extensions };
    vk_init_params.device_extensions = std::span{ device_extensions };
    vk_init_params.device_create_info_pnext = (const VkBaseInStructure*)&features2;
    vk_init_params.supported_surface_formats = std::span{ surface_formats };
    vk_init_params.surface_usage_flags = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_STORAGE_BIT;
    vk_initialize(window, vk_init_params);

    // Device properties.
    {
        VkPhysicalDeviceProperties2 physical_device_properties { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
        vkGetPhysicalDeviceProperties2(vk.physical_device, &physical_device_properties);

        printf("Device: %s\n", physical_device_properties.properties.deviceName);
        printf("Vulkan API version: %d.%d.%d\n",
            VK_VERSION_MAJOR(physical_device_properties.properties.apiVersion),
            VK_VERSION_MINOR(physical_device_properties.properties.apiVersion),
            VK_VERSION_PATCH(physical_device_properties.properties.apiVersion)
        );

        printf("\n");
        printf("VkPhysicalDeviceRayTracingPipelinePropertiesKHR:\n");
        printf("  shaderGroupHandleSize = %u\n", vk.ray_tracing_pipeline_properties.shaderGroupHandleSize);
        printf("  maxRayRecursionDepth = %u\n", vk.ray_tracing_pipeline_properties.maxRayRecursionDepth);
        printf("  maxShaderGroupStride = %u\n", vk.ray_tracing_pipeline_properties.maxShaderGroupStride);
        printf("  shaderGroupBaseAlignment = %u\n", vk.ray_tracing_pipeline_properties.shaderGroupBaseAlignment);
        printf("  maxRayDispatchInvocationCount = %u\n", vk.ray_tracing_pipeline_properties.maxRayDispatchInvocationCount);
        printf("  shaderGroupHandleAlignment = %u\n", vk.ray_tracing_pipeline_properties.shaderGroupHandleAlignment);
        printf("  maxRayHitAttributeSize = %u\n", vk.ray_tracing_pipeline_properties.maxRayHitAttributeSize);
    }

    descriptor_heap_layout.initialize();
    const uint32_t descriptor_data_size = descriptor_heap_layout.get_total_descriptor_data_size(0);
    descriptor_heap.create(descriptor_data_size);

    const std::vector<VkDescriptorSetAndBindingMappingEXT> descriptor_mappings = descriptor_heap_layout.get_descriptor_mappings();

    kernels.create_kernels(descriptor_heap_layout);
    path_tracing_renderer.initialize(descriptor_mappings, time_keeper);
    direct_lighting_renderer.initialize(descriptor_mappings, time_keeper);

    restore_resolution_dependent_resources();
    write_sampler_descriptors();

    uint8_t black[4] = { 0, 0, 0, 255 };
    black_texture = vk_create_texture(1, 1, VK_FORMAT_R8G8B8A8_UNORM, false, black, 4, "black_texture_1x1");

    // ImGui setup.
    {
        ImGui::CreateContext();
        ImGui_ImplGlfw_InitForVulkan(window, true);

        ImGui_ImplVulkan_InitInfo init_info{};
        init_info.Instance = vk.instance;
        init_info.PhysicalDevice = vk.physical_device;
        init_info.Device = vk.device;
        init_info.QueueFamily = vk.queue_family_index;
        init_info.Queue = vk.queue;
        init_info.DescriptorPool = vk.imgui_descriptor_pool;
        init_info.MinImageCount = 2;
        init_info.ImageCount = (uint32_t)vk.swapchain_info.images.size();
        init_info.UseDynamicRendering = true;
        init_info.PipelineInfoMain.PipelineRenderingCreateInfo = { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        init_info.PipelineInfoMain.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
        init_info.PipelineInfoMain.PipelineRenderingCreateInfo.pColorAttachmentFormats = &vk.surface_format.format;

        ImGui_ImplVulkan_Init(&init_info);
        ImGui::StyleColorsDark();
    }

    timer_frame = time_keeper.allocate_timer("frame");
    timer_ui = time_keeper.allocate_timer("ui");
    time_keeper.initialize_timers();
    ui.frame_time_scope = timer_frame;
}

void YAR::shutdown() {
    wait_for_reference_renderer();
    VK_CHECK(vkDeviceWaitIdle(vk.device));

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    descriptor_heap.destroy();
    kernels.destroy_kernels();

    release_resolution_dependent_resources();

    path_tracing_renderer.destroy();
    direct_lighting_renderer.destroy();

    if (gpu_scene.loaded) {
        gpu_scene.destroy();
    }
    black_texture.destroy();
    vk_shutdown();
}

void YAR::recreate_swapchain()
{
    VK_CHECK(vkDeviceWaitIdle(vk.device));
    release_resolution_dependent_resources();
    vk_destroy_swapchain();
    vk_create_swapchain(vsync_enabled());
    restore_resolution_dependent_resources();
}

void YAR::release_resolution_dependent_resources()
{
    path_tracing_renderer.destroy_resolution_dependent_resources();
    direct_lighting_renderer.destroy_resolution_dependent_resources();
}

void YAR::restore_resolution_dependent_resources()
{
    path_tracing_renderer.create_resolution_dependent_resources();
    path_tracing_renderer.activate();

    direct_lighting_renderer.create_resolution_dependent_resources();

    write_resolution_dependent_descriptors();
}

void YAR::write_resolution_dependent_descriptors()
{
    uint32_t swapchain_image_offset = descriptor_heap_layout.get_image_descriptor_offset(Image_Descriptor_Index::swapchain_first_image);
    if (vk.swapchain_info.images.size() > max_swapchain_image_descriptors) {
        error("Too many swapchain images (%u), max_swapchain_image_descriptors = %u\n",
            (uint32_t)vk.swapchain_info.images.size(), max_swapchain_image_descriptors);
    }
    for (size_t i = 0; i < vk.swapchain_info.images.size(); i++) {
        descriptor_heap.write_image_descriptor(vk.swapchain_info.images[i], vk.surface_format.format,
            VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, swapchain_image_offset);
        swapchain_image_offset += vk_image_descriptor_size();
    }
    path_tracing_renderer.write_resolution_dependent_descriptors(descriptor_heap, descriptor_heap_layout);
    direct_lighting_renderer.write_resolution_dependent_descriptors(descriptor_heap, descriptor_heap_layout);
}

void YAR::write_sampler_descriptors()
{
    VkSamplerCreateInfo sampler_create_info{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    descriptor_heap.write_sampler_descriptor(sampler_create_info, descriptor_heap_layout.image_sampler);
}

void YAR::load_project(const std::string& input_file)
{
    ASSERT(scene.type == Scene_Type::none);
    VK_CHECK(vkDeviceWaitIdle(vk.device));

    scene = load_scene(input_file);
    gpu_scene.load(scene);

    const uint32_t descriptor_data_size = descriptor_heap_layout.get_total_descriptor_data_size(uint32_t(gpu_scene.images.size()));
    if (descriptor_data_size > descriptor_heap.resource_reserved_region_offset) {
        descriptor_heap.destroy();
        descriptor_heap.create(descriptor_data_size);
        write_resolution_dependent_descriptors();
        write_sampler_descriptors();
    }
    gpu_scene.write_descriptors(descriptor_heap, descriptor_heap_layout);

    vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
        descriptor_heap.bind(command_buffer);
        kernels.patch_materials.dispatch(command_buffer);
    });

    flying_camera.initialize(scene.view_points[0], scene.z_is_up);
    ui.project_file = input_file;

    if (ui.rendering_algorithm == 0) {
        direct_lighting_renderer.activate();
    }
    else if (ui.rendering_algorithm == 1) {
        path_tracing_renderer.activate();
    }
}

void YAR::unload_project()
{
    ASSERT(scene.type != Scene_Type::none);

    wait_for_reference_renderer();
    VK_CHECK(vkDeviceWaitIdle(vk.device));

    gpu_scene.clear_descriptors(descriptor_heap, descriptor_heap_layout);
    gpu_scene.destroy();
    scene = Scene{};
    flying_camera = Flying_Camera{};
}

static double last_frame_time;

void YAR::run_frame()
{
    ui.reference_renderer_running = reference_renderer_running.load();

    if (ui.rendering_algorithm == 0) {
        timer_frame->nested_timers = {
            direct_lighting_renderer.timer_draw,
            direct_lighting_renderer.timer_tonemap,
            direct_lighting_renderer.timer_compute_copy,
            timer_ui
        };
    }
    else if (ui.rendering_algorithm == 1) {
        timer_frame->nested_timers = {
            path_tracing_renderer.timer_draw,
            path_tracing_renderer.timer_tonemap,
            path_tracing_renderer.timer_compute_copy,
            timer_ui
        };
    }

    const UI_Actions ui_actions = ui.run_imgui();

    if (last_frame_time == 0.0) { // initialize
        last_frame_time = glfwGetTime();
    }
    double current_time = glfwGetTime();
    double dt = current_time - last_frame_time;
    last_frame_time = current_time;

    if (!ImGui::GetIO().WantCaptureKeyboard) {
        if (ImGui::IsKeyDown(ImGuiKey_F1)) {
            Matrix3x4 camera_pose = flying_camera.get_camera_pose();
            FILE* f = fopen("camera.txt", "w");

            for (int i = 0; i < 3; i++) {
                fprintf(f, "%f, %f, %f, %f,\n", camera_pose.a[i][0], camera_pose.a[i][1], camera_pose.a[i][2], camera_pose.a[i][3]);
            }
            fprintf(f, "\n");

            Vector3 from, to, up;
            get_pbrt_lookat_from_camera_pose(camera_pose, scene.z_is_up, from, to, up);
            fprintf(f, "pbrt: LookAt %f %f %f  %f %f %f  %f %f %f\n",
                from.x, from.y, from.z, to.x, to.y, to.z, up.x, up.y, up.z);

            fclose(f);
        }
    }

    if (flying_camera.update(dt)) {
        path_tracing_renderer.on_camera_changed();
    }
    ui.camera_position = flying_camera.get_camera_pose().get_column(3);

    if (ui.renderer_changed) {
        if (ui.rendering_algorithm == 0) {
            direct_lighting_renderer.activate();
        }
        else if (ui.rendering_algorithm == 1) {
            path_tracing_renderer.activate();
        }
    }
    if (ui_actions.reference_render_requested) {
        start_reference_renderer();
    }
    if (ui_actions.load_project) {
        if (scene.type != Scene_Type::none) {
            unload_project();
        }
        load_project(ui.project_file);
    }
    if (ui_actions.unload_project) {
        if (scene.type != Scene_Type::none) {
            unload_project();
        }
    }
    draw_frame();
    frame_index++;
}

void YAR::draw_frame() {
    vk_begin_frame();
    time_keeper.retrieve_query_results(); // get timestamp values from the previous frame
    timer_frame->start();
    descriptor_heap.bind(vk.command_buffer);

    // Set per-frame parameters
    static_assert(sizeof(GPU_Types::Frame_Params) == 128); // Vulkan guarantees 256 min limit
    GPU_Types::Frame_Params frame_params{};
    frame_params.frame_index = frame_index;
    frame_params.swapchain_image_index = vk.swapchain_image_index;
    frame_params.camera_to_world = flying_camera.get_camera_pose();
    frame_params.viewport_size = { vk.surface_size.width, vk.surface_size.height };
    frame_params.tan_fovy_over_2 = std::tan(radians(scene.camera_fov_y / 2.f));
    frame_params.z_is_up = uint32_t(scene.z_is_up);

    VkPushDataInfoEXT push_data_info{ VK_STRUCTURE_TYPE_PUSH_DATA_INFO_EXT };
    push_data_info.offset = 0;
    push_data_info.data.address = &frame_params;
    push_data_info.data.size = sizeof(frame_params);
    vkCmdPushDataEXT(vk.command_buffer, &push_data_info);

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL);

    if (gpu_scene.loaded) {
        if (ui.rendering_algorithm == 0) {
            direct_lighting_renderer.render(kernels);
        }
        else if (ui.rendering_algorithm == 1) {
            path_tracing_renderer.render(kernels);
        }
    }

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL,
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    draw_imgui();

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);

    timer_frame->stop();
    vk_end_frame();
}

void YAR::draw_imgui()
{
    VK_TIME_SCOPE(timer_ui);

    ImGui::Render();

    VkRenderingAttachmentInfo color_attachment{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
    color_attachment.imageView = vk.swapchain_info.image_views[vk.swapchain_image_index];
    color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    if (scene.type == Scene_Type::none) {
        // Clear background if not project is loaded
        color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        color_attachment.clearValue.color.float32[3] = 1.0; // (0, 0, 0, 1)
    }
    else {
        color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    }
    color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

    VkRenderingInfo rendering_info{ VK_STRUCTURE_TYPE_RENDERING_INFO };
    rendering_info.renderArea.extent = vk.surface_size;
    rendering_info.layerCount = 1;
    rendering_info.colorAttachmentCount = 1;
    rendering_info.pColorAttachments = &color_attachment;

    vkCmdBeginRendering(vk.command_buffer, &rendering_info);
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), vk.command_buffer);
    vkCmdEndRendering(vk.command_buffer);
}

void YAR::start_reference_renderer()
{
    ASSERT(!reference_renderer_running.load());

    Reference_Renderer_Config reference_renderer_config;
    int thread_count = ui.ref_params.thread_count;
    if (!thread_count) {
        thread_count = std::max(1u, std::thread::hardware_concurrency());
    }
    reference_renderer_config.thread_count = thread_count;

    Raytracer_Config raytracer_config = scene.raytracer_config;
    int k = (int)std::ceil(std::sqrt(ui.ref_params.spp));
    raytracer_config.x_pixel_sample_count = k;
    raytracer_config.y_pixel_sample_count = k;

    Scene_Overrides overrides;
    overrides.raytracer_config = raytracer_config;
    overrides.camera_pose = flying_camera.get_camera_pose();

    // Launch rendering thread
    ASSERT(!reference_renderer_running.load());
    reference_renderer_running.store(true);
    reference_renderer_thread = std::jthread([this, reference_renderer_config, overrides]() {
        do_run_reference_renderer(reference_renderer_config, overrides);
    });
}

void YAR::do_run_reference_renderer(const Reference_Renderer_Config& reference_renderer_config, const Scene_Overrides& overrides)
{
    // Init scene context
    Scene_Context scene_ctx;
    init_scene_context(scene_ctx, scene, reference_renderer_config, overrides);

    // Render
    double variance_estimate = 0.0;
    float render_time = 0.f;
    Image image = render_scene(scene_ctx, &variance_estimate, &render_time);

    // Save results
    EXR_Write_Params write_params;
    const std::string image_filename = "image.exr";
    if (!write_openexr_image(image_filename, image, write_params)) {
        printf("Failed to save rendered image: %s\n", image_filename.c_str());
    }
    else {
        printf("Saved output image to %s\n\n", image_filename.c_str());
    }
    reference_renderer_running.store(false);
}

void YAR::wait_for_reference_renderer()
{
    if (reference_renderer_thread.joinable()) {
        reference_renderer_thread.join();
        reference_renderer_running.store(false);
    }
}
