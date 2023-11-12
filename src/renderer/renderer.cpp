#include "std.h"
#include "lib/common.h"
#include "renderer.h"

#include "geometry.h"
#include "vk.h"
#include "vk_utils.h"
#include "shaders/shared_main.hlsli"
#include "shaders/shared_light.hlsli"
#include "shaders/shared_material.hlsli"

#include "lib/matrix.h"
#include "lib/scene_loader.h"
#include "reference/reference_renderer.h"
#include "reference/scene_context.h"

#include "glfw/glfw3.h"
#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "imgui/impl/imgui_impl_vulkan.h"
#include "imgui/impl/imgui_impl_glfw.h"

static VkFormat get_depth_image_format() {
    VkFormat candidates[2] = { VK_FORMAT_D24_UNORM_S8_UINT, VK_FORMAT_D32_SFLOAT_S8_UINT };
    for (auto format : candidates) {
        VkFormatProperties props;
        vkGetPhysicalDeviceFormatProperties(vk.physical_device, format, &props);
        if ((props.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0) {
            return format;
        }
    }
    error("failed to select depth attachment format");
    return VK_FORMAT_UNDEFINED;
}

void Renderer::initialize(GLFWwindow* window, bool enable_vulkan_validation, int gpu_index) {
    Vk_Init_Params vk_init_params;
    vk_init_params.enable_validation_layer = enable_vulkan_validation;
    vk_init_params.physical_device_index = gpu_index;
    vk_init_params.vsync = ui.vsync;
    vk_initialize(window, vk_init_params);

    // Device properties.
    {
        raytrace_scene.properties = VkPhysicalDeviceRayTracingPipelinePropertiesKHR{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR };

        VkPhysicalDeviceProperties2 physical_device_properties { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
        physical_device_properties.pNext = &raytrace_scene.properties;
        vkGetPhysicalDeviceProperties2(vk.physical_device, &physical_device_properties);

        printf("Device: %s\n", physical_device_properties.properties.deviceName);
        printf("Vulkan API version: %d.%d.%d\n",
            VK_VERSION_MAJOR(physical_device_properties.properties.apiVersion),
            VK_VERSION_MINOR(physical_device_properties.properties.apiVersion),
            VK_VERSION_PATCH(physical_device_properties.properties.apiVersion)
        );

        printf("\n");
        printf("VkPhysicalDeviceRayTracingPipelinePropertiesKHR:\n");
        printf("  shaderGroupHandleSize = %u\n", raytrace_scene.properties.shaderGroupHandleSize);
        printf("  maxRayRecursionDepth = %u\n", raytrace_scene.properties.maxRayRecursionDepth);
        printf("  maxShaderGroupStride = %u\n", raytrace_scene.properties.maxShaderGroupStride);
        printf("  shaderGroupBaseAlignment = %u\n", raytrace_scene.properties.shaderGroupBaseAlignment);
        printf("  maxRayDispatchInvocationCount = %u\n", raytrace_scene.properties.maxRayDispatchInvocationCount);
        printf("  shaderGroupHandleAlignment = %u\n", raytrace_scene.properties.shaderGroupHandleAlignment);
        printf("  maxRayHitAttributeSize = %u\n", raytrace_scene.properties.maxRayHitAttributeSize);
    }

    // point sampler
    {
        VkSamplerCreateInfo create_info { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        VK_CHECK(vkCreateSampler(vk.device, &create_info, nullptr, &point_sampler));
        vk_set_debug_name(point_sampler, "point_sampler");
    }

    create_render_passes();
    apply_tone_mapping.create();
    copy_to_swapchain.create();
    restore_resolution_dependent_resources();
    create_default_textures();

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
        init_info.DescriptorPool = vk.descriptor_pool;
        init_info.MinImageCount = 2;
        init_info.ImageCount = (uint32_t)vk.swapchain_info.images.size();
        ImGui_ImplVulkan_Init(&init_info, ui_render_pass);

        ImGui::StyleColorsDark();

        vk_execute(vk.command_pools[0], vk.queue, [](VkCommandBuffer cb) {
            ImGui_ImplVulkan_CreateFontsTexture(cb);
        });
        ImGui_ImplVulkan_DestroyFontUploadObjects();
    }

    gpu_times.frame = time_keeper.allocate_time_scope("frame");
    gpu_times.draw = time_keeper.allocate_time_scope("draw");
    gpu_times.tone_map = time_keeper.allocate_time_scope("tone_map");
    gpu_times.ui = time_keeper.allocate_time_scope("ui");
    gpu_times.compute_copy = time_keeper.allocate_time_scope("compute copy");
    gpu_times.frame->child_scopes = {gpu_times.draw, gpu_times.tone_map, gpu_times.ui, gpu_times.compute_copy};
    time_keeper.initialize_time_scopes();
    
    ui.frame_time_scope = gpu_times.frame;
    ui.spp4 = &spp4;
}

void Renderer::shutdown() {
    VK_CHECK(vkDeviceWaitIdle(vk.device));

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    gpu_scene.point_lights.destroy();
    gpu_scene.point_light_count = 0;
    gpu_scene.directional_lights.destroy();
    gpu_scene.directional_light_count = 0;
    gpu_scene.diffuse_rectangular_lights.destroy();
    gpu_scene.diffuse_rectangular_light_count = 0;

    vkDestroyDescriptorSetLayout(vk.device, gpu_scene.light_descriptor_set_layout, nullptr);

    gpu_scene.lambertian_material_buffer.destroy();
    vkDestroyDescriptorSetLayout(vk.device, gpu_scene.material_descriptor_set_layout, nullptr);

    vkDestroyDescriptorSetLayout(vk.device, gpu_scene.base_descriptor_set_layout, nullptr);

    vkDestroyPipelineLayout(vk.device, gpu_scene.per_frame_pipeline_layout, nullptr);

    for (GPU_Mesh& mesh : gpu_meshes) {
        mesh.vertex_buffer.destroy();
        mesh.index_buffer.destroy();
    }
    gpu_meshes.clear();

    vkDestroySampler(vk.device, point_sampler, nullptr);

    for (Vk_Image& image : gpu_scene.images_2d)
        image.destroy();

    gpu_scene.instance_info_buffer.destroy();

    apply_tone_mapping.destroy();
    copy_to_swapchain.destroy();
    vkDestroyRenderPass(vk.device, ui_render_pass, nullptr);
    release_resolution_dependent_resources();

    if (project_loaded) {
        patch_materials.destroy();
        raytrace_scene.destroy();
    }

    vk_shutdown();
}

void Renderer::release_resolution_dependent_resources() {
    for (VkFramebuffer framebuffer : ui_framebuffers) {
        vkDestroyFramebuffer(vk.device, framebuffer, nullptr);
    }
    ui_framebuffers.resize(0);
    depth_buffer_image.destroy();
    output_image.destroy();
}

void Renderer::restore_resolution_dependent_resources() {
    // create depth buffer
    {
        VkFormat depth_format = get_depth_image_format();
        depth_buffer_image = vk_create_image(vk.surface_size.width, vk.surface_size.height, depth_format,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, "depth_buffer");

        vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
            VkImageSubresourceRange subresource_range{};
            subresource_range.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
            subresource_range.levelCount = 1;
            subresource_range.layerCount = 1;

            vk_cmd_image_barrier_for_subresource(command_buffer, depth_buffer_image.handle, subresource_range,
                VK_PIPELINE_STAGE_NONE, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                VK_PIPELINE_STAGE_NONE, 0, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
            });
    }

    // output image
    {
        output_image = vk_create_image(vk.surface_size.width, vk.surface_size.height, VK_FORMAT_R16G16B16A16_SFLOAT,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, "output_image");

        vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
            vk_cmd_image_barrier(command_buffer, output_image.handle,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_GENERAL);
        });
    }

    // imgui framebuffer
    {
        VkFramebufferCreateInfo create_info { VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        create_info.renderPass = ui_render_pass;
        create_info.attachmentCount = 1;
        create_info.width = vk.surface_size.width;
        create_info.height = vk.surface_size.height;
        create_info.layers = 1;
        ui_framebuffers.resize(vk.swapchain_info.image_views.size());
        for (int i = 0; i < (int)vk.swapchain_info.image_views.size(); i++) {
            create_info.pAttachments = &vk.swapchain_info.image_views[i];
            VK_CHECK(vkCreateFramebuffer(vk.device, &create_info, nullptr, &ui_framebuffers[i]));;
        }
    }

    if (project_loaded) {
        raytrace_scene.update_output_image_descriptor(output_image.view);
    }

    apply_tone_mapping.update_resolution_dependent_descriptors(output_image.view);
    copy_to_swapchain.update_resolution_dependent_descriptors(output_image.view);
}

void Renderer::load_project(const std::string& input_file) {
    scene = load_scene(input_file);

    flying_camera.initialize(scene.view_points[0], scene.z_is_up);

    // TODO: temp structure. Use separate buffer per attribute.
    struct GPU_Vertex {
        Vector3 position;
        Vector3 normal;
        Vector2 uv;
    };

    // Create geometry.
    gpu_meshes.resize(scene.geometries.triangle_meshes.size());
    for (int i = 0; i < (int)scene.geometries.triangle_meshes.size(); i++) {
        const Triangle_Mesh& triangle_mesh = scene.geometries.triangle_meshes[i];
        GPU_Mesh& gpu_mesh = gpu_meshes[i];

        gpu_mesh.vertex_count = (uint32_t)triangle_mesh.vertices.size();
        gpu_mesh.index_count = (uint32_t)triangle_mesh.indices.size();

        // TODO: Create separate buffers per attribute instead of single bufffer:
        // better cache coherency when working only with subset of vertex attributes,
        // also it will match Triangle_Mesh data layout, so no conversion will be needed.
        std::vector<GPU_Vertex> gpu_vertices(gpu_mesh.vertex_count);
        for (size_t k = 0; k < gpu_mesh.vertex_count; k++) {
            gpu_vertices[k].position = triangle_mesh.vertices[k];
            if (!triangle_mesh.normals.empty())
                gpu_vertices[k].normal = triangle_mesh.normals[k];
            if (!triangle_mesh.uvs.empty())
                gpu_vertices[k].uv = triangle_mesh.uvs[k];
        }

        const VkDeviceSize vertex_buffer_size = gpu_vertices.size() * sizeof(GPU_Vertex);
        VkBufferUsageFlags vertex_usage_flags = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;

        gpu_mesh.vertex_buffer = vk_create_buffer(vertex_buffer_size, vertex_usage_flags, gpu_vertices.data(), "vertex_buffer");

        const VkDeviceSize index_buffer_size = triangle_mesh.indices.size() * sizeof(triangle_mesh.indices[0]);
        VkBufferUsageFlags index_usage_flags = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;

        gpu_mesh.index_buffer = vk_create_buffer(index_buffer_size, index_usage_flags, triangle_mesh.indices.data(), "index_buffer");

        // TODO: this is wrong! render objects list should not be indexed by geometry index. 
        // Will be fixed when gpu renderer will support Render_Objects (i.e. instancing).
        int area_light_index = i - int(scene.geometries.triangle_meshes.size() - scene.lights.diffuse_rectangular_lights.size());
        if (area_light_index >= 0)
            gpu_mesh.area_light_index = area_light_index;
        else
            gpu_mesh.material = scene.objects[i].material;
    }

    // Instance buffer.
    {
        std::vector<GPU_Types::Instance_Info> instance_infos(scene.objects.size());
        for (auto [i, scene_object] : enumerate(scene.objects)) {
            instance_infos[i].material.init(scene_object.material);
            instance_infos[i].geometry.init(scene_object.geometry);
            // TODO: this should be Light_Handle not just light_index, since we could have multiple types of area lights. 
            instance_infos[i].area_light_index = scene_object.area_light.index;
            instance_infos[i].pad0 = 0.f;
            instance_infos[i].pad1 = 0.f;
            instance_infos[i].pad2 = 0.f;
            instance_infos[i].object_to_world_transform = scene_object.object_to_world_transform;
        }
        VkDeviceSize size = scene.objects.size() * sizeof(GPU_Types::Instance_Info); 
        gpu_scene.instance_info_buffer = vk_create_buffer(size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            instance_infos.data(), "instance_info_buffer");
    }

    // Materials.
    {
        gpu_scene.images_2d.reserve(gpu_scene.images_2d.size() + scene.texture_descriptors.size());
        for (const Texture_Descriptor& texture_desc : scene.texture_descriptors) {
            Vk_Image image = vk_load_texture(scene.get_resource_absolute_path(texture_desc.file_name));
            gpu_scene.images_2d.push_back(image);
        }

        std::vector<GPU_Types::Lambertian_Material> gpu_lambertian_materials(scene.materials.lambertian.size());
        for (auto[i, lambertian] : enumerate(scene.materials.lambertian)) {
            const RGB_Parameter& param = lambertian.reflectance;
            gpu_lambertian_materials[i].r = param.constant_value.r;
            gpu_lambertian_materials[i].g = param.constant_value.g;
            gpu_lambertian_materials[i].b = param.constant_value.b;
            gpu_lambertian_materials[i].albedo_texture_index = param.texture_index;
            gpu_lambertian_materials[i].u_scale = param.u_scale;
            gpu_lambertian_materials[i].v_scale = param.v_scale;
        }

        if (!gpu_lambertian_materials.empty()) {
            VkDeviceSize size = gpu_lambertian_materials.size() * sizeof(GPU_Types::Lambertian_Material);
            gpu_scene.lambertian_material_buffer = vk_create_buffer(size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                gpu_lambertian_materials.data(), "lambertian_material_buffer");
        }

        {
            gpu_scene.material_descriptor_set_layout = Descriptor_Set_Layout()
                .storage_buffer(0, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_COMPUTE_BIT) // lambertian materials
                .create("material_descriptor_set_layout");

            gpu_scene.material_descriptor_set = allocate_descriptor_set(gpu_scene.material_descriptor_set_layout);
            Descriptor_Writes(gpu_scene.material_descriptor_set)
                .storage_buffer(0, gpu_scene.lambertian_material_buffer.handle, 0, VK_WHOLE_SIZE);
        }

        {
            gpu_scene.base_descriptor_set_layout = Descriptor_Set_Layout()
                .sampled_image_array(0, (uint32_t)gpu_scene.images_2d.size(), VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR)
                .sampler(1, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR)
                .storage_buffer(2, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR) // instance buffer
                .storage_buffer_array(3, (uint32_t)gpu_meshes.size(), VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR) // index buffers
                .storage_buffer_array(4, (uint32_t)gpu_meshes.size(), VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR) // vertex buffers
                .create("base_descriptor_set_layout");

            gpu_scene.base_descriptor_set = allocate_descriptor_set(gpu_scene.base_descriptor_set_layout);

            std::vector<VkDescriptorImageInfo> image_infos(gpu_scene.images_2d.size());
            for (auto[i, image] : enumerate(gpu_scene.images_2d)) {
                image_infos[i].sampler = VK_NULL_HANDLE;
                image_infos[i].imageView = image.view;
                image_infos[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            }

            std::vector<VkDescriptorBufferInfo> vertex_buffer_infos(gpu_meshes.size());
            std::vector<VkDescriptorBufferInfo> index_buffer_infos(gpu_meshes.size());

            for (auto [i, gpu_mesh] : enumerate(gpu_meshes)) {
                vertex_buffer_infos[i].buffer = gpu_mesh.vertex_buffer.handle;
                vertex_buffer_infos[i].offset = 0;
                vertex_buffer_infos[i].range = gpu_mesh.vertex_count * sizeof(GPU_Vertex);

                index_buffer_infos[i].buffer = gpu_mesh.index_buffer.handle;
                index_buffer_infos[i].offset = 0;
                index_buffer_infos[i].range = gpu_mesh.index_count * sizeof(uint32_t);
            }

            Descriptor_Writes(gpu_scene.base_descriptor_set)
                .sampled_image_array(0, (uint32_t)gpu_scene.images_2d.size(), image_infos.data())
                .sampler(1, point_sampler)
                .storage_buffer(2, gpu_scene.instance_info_buffer.handle, 0, VK_WHOLE_SIZE)
                .storage_buffer_array(3, (uint32_t)gpu_meshes.size(), index_buffer_infos.data())
                .storage_buffer_array(4, (uint32_t)gpu_meshes.size(), vertex_buffer_infos.data());
        }
    }

    // Lights.
    {
        bool found_supported_lights = false;
        if (!scene.lights.point_lights.empty()) {
            found_supported_lights = true;
            std::vector<GPU_Types::Point_Light> lights(scene.lights.point_lights.size());
            for (auto [i, data] : enumerate(scene.lights.point_lights)) {
                lights[i].init(data);
            }
            gpu_scene.point_lights = vk_create_buffer(lights.size() * sizeof(lights[0]),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                lights.data(), "point_light_buffer");
            gpu_scene.point_light_count = (uint32_t)lights.size();
        }
        if (!scene.lights.directional_lights.empty()) {
            found_supported_lights = true;
            std::vector<GPU_Types::Directional_Light> lights(scene.lights.directional_lights.size());
            for (auto [i, data] : enumerate(scene.lights.directional_lights)) {
                lights[i].init(data);
            }
            gpu_scene.directional_lights = vk_create_buffer(lights.size() * sizeof(lights[0]),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                lights.data(), "directional_light_buffer");
            gpu_scene.directional_light_count = (uint32_t)lights.size();
        }
        if (!scene.lights.diffuse_rectangular_lights.empty()) {
            found_supported_lights = true;
            std::vector<GPU_Types::Diffuse_Rectangular_Light> lights(scene.lights.diffuse_rectangular_lights.size());
            for (auto [i, data] : enumerate(scene.lights.diffuse_rectangular_lights)) {
                lights[i].init(data);
            }
            gpu_scene.diffuse_rectangular_lights = vk_create_buffer(lights.size() * sizeof(lights[0]),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                lights.data(), "diffuse_rectangular_light_buffer");
            gpu_scene.diffuse_rectangular_light_count = (uint32_t)lights.size();
        }
        if (scene.lights.has_environment_light) {
            printf("Scene contains environment light. Environment lights are not suported yet.\n");
        }
        // Add default directional light if no supported lights were found
        if (!found_supported_lights) {
            Directional_Light scene_light;
            scene_light.direction = Vector3(1, 1, 1).normalized();
            scene_light.irradiance = ColorRGB(5, 5, 5);

            GPU_Types::Directional_Light gpu_light;
            gpu_light.init(scene_light);
            gpu_scene.directional_lights = vk_create_buffer(sizeof(GPU_Types::Directional_Light),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                &gpu_light, "directional_light_buffer");
            gpu_scene.directional_light_count = 1;
            printf("No supported lights found. Added default directional light\n");
        }

        gpu_scene.light_descriptor_set_layout = Descriptor_Set_Layout()
            .storage_buffer(POINT_LIGHT_BINDING, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR)
            .storage_buffer(DIRECTIONAL_LIGHT_BINDING, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR)
            .storage_buffer(DIFFUSE_RECTANGULAR_LIGHT_BINDING, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR)
            .create("light_descriptor_set_layout");

        gpu_scene.light_descriptor_set = allocate_descriptor_set(gpu_scene.light_descriptor_set_layout);
        Descriptor_Writes(gpu_scene.light_descriptor_set)
            .storage_buffer(POINT_LIGHT_BINDING, gpu_scene.point_lights.handle, 0, VK_WHOLE_SIZE)
            .storage_buffer(DIRECTIONAL_LIGHT_BINDING, gpu_scene.directional_lights.handle, 0, VK_WHOLE_SIZE)
            .storage_buffer(DIFFUSE_RECTANGULAR_LIGHT_BINDING, gpu_scene.diffuse_rectangular_lights.handle, 0, VK_WHOLE_SIZE);
    }

    kernel_context.base_descriptor_set_layout = gpu_scene.base_descriptor_set_layout;
    kernel_context.light_descriptor_set_layout = gpu_scene.light_descriptor_set_layout;
    kernel_context.material_descriptor_set_layout = gpu_scene.material_descriptor_set_layout;

    gpu_scene.per_frame_pipeline_layout = create_pipeline_layout(
        {
            gpu_scene.base_descriptor_set_layout,
            gpu_scene.material_descriptor_set_layout,
            gpu_scene.light_descriptor_set_layout,
        },
        {
            VkPushConstantRange{VK_SHADER_STAGE_ALL, 0, Compatible_Layout_Push_Constant_Count * sizeof(uint32_t)}
        },
        "per_frame_pipeline_layout"
    );

    patch_materials.create(gpu_scene.material_descriptor_set_layout);
    vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
        patch_materials.dispatch(command_buffer, gpu_scene.material_descriptor_set);
    });

    raytrace_scene.create(kernel_context, scene, gpu_meshes);
    raytrace_scene.update_output_image_descriptor(output_image.view);
    raytrace_scene.update_point_lights(gpu_scene.point_light_count);
    raytrace_scene.update_directional_lights(gpu_scene.directional_light_count);
    raytrace_scene.update_diffuse_rectangular_lights(gpu_scene.diffuse_rectangular_light_count);

    Descriptor_Writes(gpu_scene.light_descriptor_set).storage_buffer(POINT_LIGHT_BINDING, gpu_scene.point_lights.handle, 0, VK_WHOLE_SIZE);
    Descriptor_Writes(gpu_scene.light_descriptor_set).storage_buffer(DIRECTIONAL_LIGHT_BINDING, gpu_scene.directional_lights.handle, 0, VK_WHOLE_SIZE);
    Descriptor_Writes(gpu_scene.light_descriptor_set).storage_buffer(DIFFUSE_RECTANGULAR_LIGHT_BINDING, gpu_scene.diffuse_rectangular_lights.handle, 0, VK_WHOLE_SIZE);

    project_loaded = true;
}

static double last_frame_time;

void Renderer::run_frame() {
    ui.run_imgui();

    if (last_frame_time == 0.0) { // initialize
        last_frame_time = glfwGetTime();
    }
    double current_time = glfwGetTime();
    double dt = current_time - last_frame_time;
    last_frame_time = current_time;

    if (!ImGui::GetIO().WantCaptureKeyboard) {
        if (ImGui::IsKeyDown(GLFW_KEY_F1)) {
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

    flying_camera.update(dt);
    ui.camera_position = flying_camera.get_camera_pose().get_column(3);

    if (project_loaded) {
        raytrace_scene.update_camera_transform(flying_camera.get_camera_pose());
    }

    if (ui.ui_result.reference_render_requested) {
        start_reference_renderer();
        ui.ui_result.reference_render_requested = false;
    }

    draw_frame();
}

void Renderer::create_render_passes() {
    // UI render pass.
    {
        VkAttachmentDescription attachments[1] = {};
        attachments[0].format           = vk.surface_format.format;
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
}

void Renderer::create_default_textures() {
    ASSERT(gpu_scene.images_2d.empty());
    gpu_scene.images_2d.resize(Predefined_Texture_Count);

    uint8_t black[4] = {0, 0, 0, 255};
    gpu_scene.images_2d[Black_2D_Texture_Index] = vk_create_texture(1, 1, VK_FORMAT_R8G8B8A8_UNORM, false, black, 4, "black_texture_1x1");

    uint8_t white[4] = {255, 255, 255, 255};
    gpu_scene.images_2d[White_2D_Texture_Index] = vk_create_texture(1, 1, VK_FORMAT_R8G8B8A8_UNORM, false, white, 4, "white_texture_1x1");
}

void Renderer::draw_frame() {
    vk_begin_frame();
    time_keeper.retrieve_query_results(); // get timestamp values from the previous frame
    gpu_times.frame->begin();

    if (project_loaded) {
        VkDescriptorSet per_frame_sets[] = {
            gpu_scene.base_descriptor_set,
            gpu_scene.material_descriptor_set,
            gpu_scene.light_descriptor_set
        };
        vkCmdBindDescriptorSets(vk.command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, gpu_scene.per_frame_pipeline_layout, 0, (uint32_t)std::size(per_frame_sets), per_frame_sets, 0, nullptr);
        draw_raytraced_image();
    }

    tone_mapping();

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL);

    copy_output_image_to_swapchain();

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL,
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    draw_imgui();

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);

    gpu_times.frame->end();
    vk_end_frame();
}

void Renderer::draw_raytraced_image() {
    GPU_TIME_SCOPE(gpu_times.draw);
    raytrace_scene.dispatch(scene.camera_fov_y, spp4, scene.z_is_up);
}

void Renderer::tone_mapping()
{
    GPU_TIME_SCOPE(gpu_times.tone_map);
    apply_tone_mapping.dispatch();
}

void Renderer::draw_imgui() {
    GPU_TIME_SCOPE(gpu_times.ui);

    ImGui::Render();

    VkRenderPassBeginInfo render_pass_begin_info{ VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    render_pass_begin_info.renderPass = ui_render_pass;
    render_pass_begin_info.framebuffer = ui_framebuffers[vk.swapchain_image_index];
    render_pass_begin_info.renderArea.extent = vk.surface_size;

    vkCmdBeginRenderPass(vk.command_buffer, &render_pass_begin_info, VK_SUBPASS_CONTENTS_INLINE);
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), vk.command_buffer);
    vkCmdEndRenderPass(vk.command_buffer);
}

void Renderer::copy_output_image_to_swapchain() {
    GPU_TIME_SCOPE(gpu_times.compute_copy);
    copy_to_swapchain.dispatch();
}

void Renderer::start_reference_renderer()
{
    Renderer_Configuration renderer_config;
    int thread_count = ui.ref_params.thread_count;
    if (!thread_count) {
        thread_count = std::max(1u, std::thread::hardware_concurrency());
    }
    renderer_config.thread_count = thread_count;

    int k = (int)std::ceil(std::sqrt(ui.ref_params.spp));
    scene.raytracer_config.x_pixel_sample_count = k;
    scene.raytracer_config.y_pixel_sample_count = k;

    Scene_Context scene_ctx;
    init_scene_context(scene, renderer_config, scene_ctx);

    scene_ctx.camera = Camera(flying_camera.get_camera_pose(), Vector2(scene.film_resolution), scene.camera_fov_y, scene.z_is_up);

    double variance_estimate = 0.0;
    float render_time = 0.f;
    Image image = render_scene(scene_ctx, &variance_estimate, &render_time);

    EXR_Write_Params write_params;
    const std::string image_filename = "image.exr";
    if (!write_openexr_image(image_filename, image, write_params)) {
        printf("Failed to save rendered image: %s\n", image_filename.c_str());
    }
    else {
        printf("Saved output image to %s\n\n", image_filename.c_str());
    }
}
