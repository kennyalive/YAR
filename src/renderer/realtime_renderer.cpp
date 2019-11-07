#include "std.h"
#include "lib/common.h"
#include "realtime_renderer.h"
#include "vk.h"
#include "utils.h"

#include "lib/matrix.h"
#include "reference/reference_renderer.h"

#include "shaders/shared_main.h"
#include "shaders/shared_light.h"

#include "glfw/glfw3.h"
#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "imgui/imgui_impl_vulkan.h"
#include "imgui/imgui_impl_glfw.h"

#include <cinttypes>

void Realtime_Renderer::initialize(Vk_Create_Info vk_create_info, GLFWwindow* window) {
    vk_initialize(window, vk_create_info);

    // Device properties.
    {
        VkPhysicalDeviceProperties2 physical_device_properties { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
        if (vk.raytracing_supported) {
            raytrace_scene.properties = VkPhysicalDeviceRayTracingPropertiesNV { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PROPERTIES_NV };
            physical_device_properties.pNext = &raytrace_scene.properties;
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
            printf("  shaderGroupHandleSize = %u\n", raytrace_scene.properties.shaderGroupHandleSize);
            printf("  maxRecursionDepth = %u\n", raytrace_scene.properties.maxRecursionDepth);
            printf("  maxShaderGroupStride = %u\n", raytrace_scene.properties.maxShaderGroupStride);
            printf("  shaderGroupBaseAlignment = %u\n", raytrace_scene.properties.shaderGroupBaseAlignment);
            printf("  maxGeometryCount = %" PRIu64 "\n", raytrace_scene.properties.maxGeometryCount);
            printf("  maxInstanceCount = %" PRIu64 "\n", raytrace_scene.properties.maxInstanceCount);
            printf("  maxTriangleCount = %" PRIu64 "\n", raytrace_scene.properties.maxTriangleCount);
            printf("  maxDescriptorSetAccelerationStructures = %u\n", raytrace_scene.properties.maxDescriptorSetAccelerationStructures);
        }
        else {
            raytracing = false;
        }
    }

    create_render_passes();
    copy_to_swapchain.create();
    restore_resolution_dependent_resources();
    create_default_textures();

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

    gpu_times.frame = time_keeper.allocate_time_scope();
    gpu_times.draw = time_keeper.allocate_time_scope();
    gpu_times.ui = time_keeper.allocate_time_scope();
    gpu_times.compute_copy = time_keeper.allocate_time_scope();
    time_keeper.initialize_time_scopes();
}

void Realtime_Renderer::shutdown() {
    VK_CHECK(vkDeviceWaitIdle(vk.device));

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    gpu_scene.point_lights.destroy();
    gpu_scene.diffuse_rectangular_lights.destroy();
    vkDestroyDescriptorSetLayout(vk.device, gpu_scene.light_descriptor_set_layout, nullptr);

    gpu_scene.lambertian_material_buffer.destroy();
    vkDestroyDescriptorSetLayout(vk.device, gpu_scene.material_descriptor_set_layout, nullptr);

    vkDestroyDescriptorSetLayout(vk.device, gpu_scene.image_descriptor_set_layout, nullptr);

    vkDestroyPipelineLayout(vk.device, gpu_scene.per_frame_pipeline_layout, nullptr);

    for (GPU_Mesh& mesh : gpu_meshes) {
        mesh.vertex_buffer.destroy();
        mesh.index_buffer.destroy();
    }
    gpu_meshes.clear();

    for (Vk_Image& image : gpu_scene.images_2d)
        image.destroy();

    copy_to_swapchain.destroy();
    vkDestroyRenderPass(vk.device, raster_render_pass, nullptr);
    vkDestroyRenderPass(vk.device, ui_render_pass, nullptr);
    release_resolution_dependent_resources();

    if (project_loaded) {
        patch_materials.destroy();
        draw_mesh.destroy();
        if (vk.raytracing_supported)
            raytrace_scene.destroy();
    }
    
    vk_shutdown();
}

void Realtime_Renderer::release_resolution_dependent_resources() {
    vkDestroyFramebuffer(vk.device, raster_framebuffer, nullptr);
    raster_framebuffer = VK_NULL_HANDLE;

    vkDestroyFramebuffer(vk.device, ui_framebuffer, nullptr);
    ui_framebuffer = VK_NULL_HANDLE;

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
        else {
            vk_execute(vk.command_pool, vk.queue, [this](VkCommandBuffer command_buffer) {
                vk_cmd_image_barrier(command_buffer, output_image.handle,
                    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,  VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                    0,                                  0,
                    VK_IMAGE_LAYOUT_UNDEFINED,          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            });
        }
    }

    // rasterizer framebuffer
    {
        VkImageView attachments[] = {output_image.view, vk.depth_info.image_view};

        VkFramebufferCreateInfo create_info { VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        create_info.renderPass      = raster_render_pass;
        create_info.attachmentCount = (uint32_t)std::size(attachments);
        create_info.pAttachments    = attachments;
        create_info.width           = vk.surface_size.width;
        create_info.height          = vk.surface_size.height;
        create_info.layers          = 1;

        VK_CHECK(vkCreateFramebuffer(vk.device, &create_info, nullptr, &raster_framebuffer));
        vk_set_debug_name(raster_framebuffer, "color_depth_framebuffer");

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

    if (project_loaded) {
        if (vk.raytracing_supported)
            raytrace_scene.update_output_image_descriptor(output_image.view);
    }

    copy_to_swapchain.update_resolution_dependent_descriptors(output_image.view);
}

void Realtime_Renderer::load_project(const std::string& yar_file_name) {
    project = initialize_project(yar_file_name);
    scene = load_scene(project);

    const std::string project_dir = get_directory(get_resource_path(project.scene_path));

    flying_camera.initialize(scene.view_points[0]);

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

        gpu_mesh.model_vertex_count = static_cast<uint32_t>(triangle_mesh.vertices.size());
        gpu_mesh.model_index_count = static_cast<uint32_t>(triangle_mesh.indices.size());

        // TODO: Create separate buffers per attribute instead of single bufffer:
        // better cache coherency when working only with subset of vertex attributes,
        // also it will match Triangle_Mesh data layout, so no conversion will be needed.
        std::vector<GPU_Vertex> gpu_vertices(gpu_mesh.model_vertex_count);
        for (int k = 0; k < gpu_mesh.model_vertex_count; k++) {
            gpu_vertices[k].position = triangle_mesh.vertices[k];
            gpu_vertices[k].normal = triangle_mesh.normals[k];
            if (!triangle_mesh.uvs.empty())
                gpu_vertices[k].uv = triangle_mesh.uvs[k];
        }

        const VkDeviceSize vertex_buffer_size = gpu_vertices.size() * sizeof(GPU_Vertex);
        gpu_mesh.vertex_buffer = vk_create_buffer(vertex_buffer_size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, gpu_vertices.data(), "vertex_buffer");

        const VkDeviceSize index_buffer_size = triangle_mesh.indices.size() * sizeof(triangle_mesh.indices[0]);
        gpu_mesh.index_buffer = vk_create_buffer(index_buffer_size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, triangle_mesh.indices.data(), "index_buffer");

        // TODO: this is wrong! render objects list should not be indexed by geometry index. 
        // Will be fixed when gpu renderer will support Render_Objects (i.e. instancing).
        int area_light_index = i - int(scene.geometries.triangle_meshes.size() - scene.lights.diffuse_rectangular_lights.size());
        if (area_light_index >= 0)
            gpu_mesh.area_light_index = area_light_index;
        else
            gpu_mesh.material = scene.render_objects[i].material;
    }

    // Materials.
    {
        gpu_scene.images_2d.reserve(gpu_scene.images_2d.size() + scene.materials.texture_names.size());
        for (const std::string& texture_name : scene.materials.texture_names) {
            Vk_Image image = vk_load_texture(fs::path(project_dir).concat(texture_name).string());
            gpu_scene.images_2d.push_back(image);
        }

        VkDeviceSize size = scene.materials.lambertian.size() * sizeof(Lambertian_Material);
        gpu_scene.lambertian_material_buffer = vk_create_buffer(size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            scene.materials.lambertian.data(), "lambertian_material_buffer");

        {
            gpu_scene.material_descriptor_set_layout = Descriptor_Set_Layout()
                .storage_buffer(0, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV | VK_SHADER_STAGE_COMPUTE_BIT) // lambertian materials
                .create("material_descriptor_set_layout");

            VkDescriptorSetAllocateInfo alloc_info{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            alloc_info.descriptorPool = vk.descriptor_pool;
            alloc_info.descriptorSetCount = 1;
            alloc_info.pSetLayouts = &gpu_scene.material_descriptor_set_layout;
            VK_CHECK(vkAllocateDescriptorSets(vk.device, &alloc_info, &gpu_scene.material_descriptor_set));

            Descriptor_Writes(gpu_scene.material_descriptor_set)
                .storage_buffer(0, gpu_scene.lambertian_material_buffer.handle, 0, VK_WHOLE_SIZE);
        }

        {
            gpu_scene.image_descriptor_set_layout = Descriptor_Set_Layout()
                .sample_image_array(0, (uint32_t)gpu_scene.images_2d.size(), VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV)
                .sampler(1, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV)
                .create("image_descriptor_set_layout");

            VkDescriptorSetAllocateInfo alloc_info{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            alloc_info.descriptorPool = vk.descriptor_pool;
            alloc_info.descriptorSetCount = 1;
            alloc_info.pSetLayouts = &gpu_scene.image_descriptor_set_layout;
            VK_CHECK(vkAllocateDescriptorSets(vk.device, &alloc_info, &gpu_scene.image_descriptor_set));

            std::vector<VkDescriptorImageInfo> image_infos(gpu_scene.images_2d.size());
            for (auto[i, image] : enumerate(gpu_scene.images_2d)) {
                image_infos[i].sampler = VK_NULL_HANDLE;
                image_infos[i].imageView = image.view;
                image_infos[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            }
            Descriptor_Writes(gpu_scene.image_descriptor_set)
                .sampled_image_array(0, (uint32_t)gpu_scene.images_2d.size(), image_infos.data())
                .sampler(1, copy_to_swapchain.point_sampler);
        }
    }

    // Lights.
    {
        if (!scene.lights.point_lights.empty()) {
            std::vector<GPU_Types::Point_Light> lights(scene.lights.point_lights.size());
            for (auto[i, data] : enumerate(scene.lights.point_lights))
                lights[i].init(data);

            gpu_scene.point_lights = vk_create_buffer(lights.size() * sizeof(lights[0]),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                lights.data(), "point_light_buffer");
        }
        if (!scene.lights.diffuse_rectangular_lights.empty()) {
            std::vector<GPU_Types::Diffuse_Rectangular_Light> lights(scene.lights.diffuse_rectangular_lights.size());
            for (auto[i, data] : enumerate(scene.lights.diffuse_rectangular_lights))
                lights[i].init(data);

            gpu_scene.diffuse_rectangular_lights = vk_create_buffer(lights.size() * sizeof(lights[0]),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                lights.data(), "diffuse_rectangular_light_buffer");
        }
        {
        gpu_scene.light_descriptor_set_layout = Descriptor_Set_Layout()
            .storage_buffer(POINT_LIGHT_BINDING, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV)
            .storage_buffer(DIFFUSE_RECTANGULAR_LIGHT_BINDING, VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV)
            .create("light_descriptor_set_layout");

        VkDescriptorSetAllocateInfo alloc_info{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        alloc_info.descriptorPool = vk.descriptor_pool;
        alloc_info.descriptorSetCount = 1;
        alloc_info.pSetLayouts = &gpu_scene.light_descriptor_set_layout;
        VK_CHECK(vkAllocateDescriptorSets(vk.device, &alloc_info, &gpu_scene.light_descriptor_set));

        Descriptor_Writes(gpu_scene.light_descriptor_set)
            .storage_buffer(POINT_LIGHT_BINDING, gpu_scene.point_lights.handle, 0, VK_WHOLE_SIZE)
            .storage_buffer(DIFFUSE_RECTANGULAR_LIGHT_BINDING, gpu_scene.diffuse_rectangular_lights.handle, 0, VK_WHOLE_SIZE);
        }
    }

    kernel_context.light_descriptor_set_layout = gpu_scene.light_descriptor_set_layout;
    kernel_context.material_descriptor_set_layout = gpu_scene.material_descriptor_set_layout;
    kernel_context.image_descriptor_set_layout = gpu_scene.image_descriptor_set_layout;

    // Per-frame pipeline layout.
    {
        VkDescriptorSetLayout set_layouts[] = {
            gpu_scene.image_descriptor_set_layout,
            gpu_scene.material_descriptor_set_layout,
            gpu_scene.light_descriptor_set_layout,
        };

        VkPushConstantRange push_constant_ranges[1];
        push_constant_ranges[0].stageFlags = VK_SHADER_STAGE_ALL;
        push_constant_ranges[0].offset = 0;
        push_constant_ranges[0].size = Compatible_Layout_Push_Constant_Count * sizeof(uint32_t);

        VkPipelineLayoutCreateInfo create_info{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        create_info.setLayoutCount = (uint32_t)std::size(set_layouts);
        create_info.pSetLayouts = set_layouts;
        create_info.pushConstantRangeCount = (uint32_t)std::size(push_constant_ranges);
        create_info.pPushConstantRanges = push_constant_ranges;

        VK_CHECK(vkCreatePipelineLayout(vk.device, &create_info, nullptr, &gpu_scene.per_frame_pipeline_layout));
        vk_set_debug_name(gpu_scene.per_frame_pipeline_layout, "per_frame_pipeline_layout");
    }

    patch_materials.create(gpu_scene.material_descriptor_set_layout);
    vk_execute(vk.command_pool, vk.queue, [this](VkCommandBuffer command_buffer) {
        patch_materials.dispatch(command_buffer, gpu_scene.material_descriptor_set);
    });

    draw_mesh.create(kernel_context, raster_render_pass, scene.front_face_has_clockwise_winding);
    draw_mesh.update_point_lights((int)scene.lights.point_lights.size());
    draw_mesh.update_diffuse_rectangular_lights((int)scene.lights.diffuse_rectangular_lights.size());

    if (vk.raytracing_supported) {
        raytrace_scene.create(kernel_context, scene, gpu_meshes);
        raytrace_scene.update_output_image_descriptor(output_image.view);
        raytrace_scene.update_point_lights((int)scene.lights.point_lights.size());
        raytrace_scene.update_diffuse_rectangular_lights((int)scene.lights.diffuse_rectangular_lights.size());
    }

    Descriptor_Writes(gpu_scene.light_descriptor_set).storage_buffer(POINT_LIGHT_BINDING, gpu_scene.point_lights.handle, 0, VK_WHOLE_SIZE);
    Descriptor_Writes(gpu_scene.light_descriptor_set).storage_buffer(DIFFUSE_RECTANGULAR_LIGHT_BINDING, gpu_scene.diffuse_rectangular_lights.handle, 0, VK_WHOLE_SIZE);

    project_loaded = true;
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

    if (!ImGui::GetIO().WantCaptureKeyboard) {
        if (ImGui::IsKeyDown(GLFW_KEY_F1)) {
            Matrix3x4 camera_pose = flying_camera.get_camera_pose();
            FILE* f = fopen("camera.txt", "w");
            for (int i = 0; i < 3; i++)
                fprintf(f, "%ff, %ff, %ff, %ff,\n", camera_pose.a[i][0], camera_pose.a[i][1], camera_pose.a[i][2], camera_pose.a[i][3]);
            fclose(f);
        }
    }

    flying_camera.update(dt);

    if (project_loaded)
        draw_mesh.update(flying_camera.get_view_transform(), scene.fovy);

    if (project_loaded && vk.raytracing_supported)
        raytrace_scene.update_camera_transform(flying_camera.get_camera_pose());

    draw_frame();
}

void Realtime_Renderer::create_render_passes() {
    // Render pass for rasterization renderer.
    {
        VkAttachmentDescription attachments[2] = {};
        attachments[0].format           = VK_FORMAT_R16G16B16A16_SFLOAT;
        attachments[0].samples          = VK_SAMPLE_COUNT_1_BIT;
        attachments[0].loadOp           = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[0].storeOp          = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[0].stencilLoadOp    = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[0].stencilStoreOp   = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[0].initialLayout    = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[0].finalLayout      = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        attachments[1].format           = vk.depth_info.format;
        attachments[1].samples          = VK_SAMPLE_COUNT_1_BIT;
        attachments[1].loadOp           = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[1].storeOp          = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].stencilLoadOp    = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[1].stencilStoreOp   = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].initialLayout    = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        attachments[1].finalLayout      = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference color_attachment_ref;
        color_attachment_ref.attachment = 0;
        color_attachment_ref.layout     = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentReference depth_attachment_ref;
        depth_attachment_ref.attachment = 1;
        depth_attachment_ref.layout     = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint       = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount    = 1;
        subpass.pColorAttachments       = &color_attachment_ref;
        subpass.pDepthStencilAttachment = &depth_attachment_ref;

        VkRenderPassCreateInfo create_info{ VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
        create_info.attachmentCount = (uint32_t)std::size(attachments);
        create_info.pAttachments = attachments;
        create_info.subpassCount = 1;
        create_info.pSubpasses = &subpass;

        VK_CHECK(vkCreateRenderPass(vk.device, &create_info, nullptr, &raster_render_pass));
        vk_set_debug_name(raster_render_pass, "color_depth_render_pass");
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
}

void Realtime_Renderer::create_default_textures() {
    ASSERT(gpu_scene.images_2d.empty());
    gpu_scene.images_2d.resize(Predefined_Texture_Count);

    uint8_t black[4] = {0, 0, 0, 255};
    gpu_scene.images_2d[Black_2D_Texture_Index] = vk_create_texture(1, 1, VK_FORMAT_R8G8B8A8_UNORM, false, black, 4, "black_texture_1x1");

    uint8_t white[4] = {255, 255, 255, 255};
    gpu_scene.images_2d[White_2D_Texture_Index] = vk_create_texture(1, 1, VK_FORMAT_R8G8B8A8_UNORM, false, white, 4, "white_texture_1x1");
}

void Realtime_Renderer::draw_frame() {
    vk_begin_frame();
    time_keeper.retrieve_query_results(); // get timestamp values from previous frame (in current implementation it's alredy finished)
    gpu_times.frame->begin();

    if (raytracing && ui_result.raytracing_toggled) {
        vk_cmd_image_barrier(vk.command_buffer, output_image.handle,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            0, VK_ACCESS_SHADER_WRITE_BIT,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);
    }

    if (project_loaded) {
        if (raytracing) {
            VkDescriptorSet per_frame_sets[] = {
                gpu_scene.image_descriptor_set,
                gpu_scene.material_descriptor_set,
                gpu_scene.light_descriptor_set
            };
            vkCmdBindDescriptorSets(vk.command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_NV, gpu_scene.per_frame_pipeline_layout, 0, (uint32_t)std::size(per_frame_sets), per_frame_sets, 0, nullptr);
            draw_raytraced_image();
        }
        else {
            VkDescriptorSet per_frame_sets[] = {
                gpu_scene.image_descriptor_set,
                gpu_scene.material_descriptor_set,
                gpu_scene.light_descriptor_set
            };
            vkCmdBindDescriptorSets(vk.command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, gpu_scene.per_frame_pipeline_layout, 0, (uint32_t)std::size(per_frame_sets), per_frame_sets, 0, nullptr);
            draw_rasterized_image();
        }
    }

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
    render_pass_begin_info.renderPass        = raster_render_pass;
    render_pass_begin_info.framebuffer       = raster_framebuffer;
    render_pass_begin_info.renderArea.extent = vk.surface_size;
    render_pass_begin_info.clearValueCount   = (uint32_t)std::size(clear_values);
    render_pass_begin_info.pClearValues      = clear_values;

    vkCmdBeginRenderPass(vk.command_buffer, &render_pass_begin_info, VK_SUBPASS_CONTENTS_INLINE);
    draw_mesh.bind_sets_and_pipeline();

    for (int i = 0; i < (int)scene.render_objects.size(); i++) {
        const Render_Object& render_object = scene.render_objects[i];
        // Skip objects that we don't support yet...
        if (render_object.geometry.type != Geometry_Type::triangle_mesh)
            continue;
        if (render_object.area_light != Null_Light && render_object.area_light.type != Light_Type::diffuse_rectangular)
            continue;

        const GPU_Mesh& gpu_mesh = gpu_meshes[render_object.geometry.index];

        GPU_Types::Instance_Info instance_info;
        instance_info.material.init(render_object.material);
        instance_info.geometry.init(render_object.geometry);
        instance_info.area_light_index = render_object.area_light.index;
        instance_info.pad0 = 0.f;
        instance_info.pad1 = 0.f;
        instance_info.pad2 = 0.f;
        instance_info.object_to_world_transform = render_object.object_to_world_transform;

        draw_mesh.dispatch(gpu_mesh, instance_info);
    }

    vkCmdEndRenderPass(vk.command_buffer);
}

void Realtime_Renderer::draw_raytraced_image() {
    GPU_TIME_SCOPE(gpu_times.draw);
    raytrace_scene.dispatch(scene.fovy, spp4);
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

    if (raytracing) {
        vk_cmd_image_barrier(vk.command_buffer, output_image.handle,
            VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_NV,   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_SHADER_WRITE_BIT,             VK_ACCESS_SHADER_READ_BIT,
            VK_IMAGE_LAYOUT_GENERAL,                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    copy_to_swapchain.dispatch();

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

            ImGui::Separator();
            if (ImGui::Button("Render reference image"))
                start_reference_renderer();

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

void Realtime_Renderer::start_reference_renderer() {
    const std::string temp_project_name = "temp.yar";

    YAR_Project temp_project = project;
    temp_project.image_resolution = Vector2i{(int)vk.surface_size.width, (int)vk.surface_size.height};
    temp_project.camera_to_world = flying_camera.get_camera_pose();
    save_yar_file(temp_project_name, temp_project);

#ifdef _WIN32
    std::string temp_project_path = get_resource_path(temp_project_name);
    char cmd_line[256];
    sprintf_s(cmd_line, "RAY.exe \"%s\"", temp_project_path.c_str());

    STARTUPINFOA si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};

    BOOL result = ::CreateProcessA(
        NULL, cmd_line,
        NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi);

    if (!result) {
        ::MessageBoxA(NULL, "Failed to run RAY.exe", "Error", MB_OK);
    }
    else {
        ::CloseHandle(pi.hProcess);
        ::CloseHandle(pi.hThread);
    }
#else
#error Unsupported platform
#endif
}
