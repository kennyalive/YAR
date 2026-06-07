#include "std.h"
#include "lib/common.h"
#include "renderer.h"

#include "geometry.h"
#include "vk.h"
#include "shaders/shared.slang"

#include "lib/matrix.h"
#include "lib/scene_loader.h"
#include "reference/reference_renderer.h"
#include "reference/scene_context.h"

#include "glfw/glfw3.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_vulkan.h"
#include "imgui/imgui_impl_glfw.h"

const VkFormat output_image_format = VK_FORMAT_R16G16B16A16_SFLOAT;

void Renderer::initialize(GLFWwindow* window, int gpu_index) {
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
    std::array surface_formats = {
        VK_FORMAT_R8G8B8A8_UNORM,
        VK_FORMAT_B8G8R8A8_UNORM
    };

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
        descriptor_heap_properties = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_HEAP_PROPERTIES_EXT };

        direct_lighting.properties = VkPhysicalDeviceRayTracingPipelinePropertiesKHR{
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR };
        direct_lighting.properties.pNext = &descriptor_heap_properties;

        VkPhysicalDeviceProperties2 physical_device_properties { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
        physical_device_properties.pNext = &direct_lighting.properties;
        vkGetPhysicalDeviceProperties2(vk.physical_device, &physical_device_properties);

        path_tracing.properties = direct_lighting.properties;

        printf("Device: %s\n", physical_device_properties.properties.deviceName);
        printf("Vulkan API version: %d.%d.%d\n",
            VK_VERSION_MAJOR(physical_device_properties.properties.apiVersion),
            VK_VERSION_MINOR(physical_device_properties.properties.apiVersion),
            VK_VERSION_PATCH(physical_device_properties.properties.apiVersion)
        );

        printf("\n");
        printf("VkPhysicalDeviceRayTracingPipelinePropertiesKHR:\n");
        printf("  shaderGroupHandleSize = %u\n", direct_lighting.properties.shaderGroupHandleSize);
        printf("  maxRayRecursionDepth = %u\n", direct_lighting.properties.maxRayRecursionDepth);
        printf("  maxShaderGroupStride = %u\n", direct_lighting.properties.maxShaderGroupStride);
        printf("  shaderGroupBaseAlignment = %u\n", direct_lighting.properties.shaderGroupBaseAlignment);
        printf("  maxRayDispatchInvocationCount = %u\n", direct_lighting.properties.maxRayDispatchInvocationCount);
        printf("  shaderGroupHandleAlignment = %u\n", direct_lighting.properties.shaderGroupHandleAlignment);
        printf("  maxRayHitAttributeSize = %u\n", direct_lighting.properties.maxRayHitAttributeSize);
    }

    descriptor_heap.create(descriptor_heap_properties);
    descriptors.initialize(descriptor_heap);

    apply_tone_mapping.create(descriptors);
    copy_to_swapchain.create(descriptors);
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
        init_info.DescriptorPool = vk.imgui_descriptor_pool;
        init_info.MinImageCount = 2;
        init_info.ImageCount = (uint32_t)vk.swapchain_info.images.size();
        init_info.UseDynamicRendering = true;
        init_info.PipelineRenderingCreateInfo = { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        init_info.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
        init_info.PipelineRenderingCreateInfo.pColorAttachmentFormats = &vk.surface_format.format;

        ImGui_ImplVulkan_Init(&init_info);
        ImGui::StyleColorsDark();
        ImGui_ImplVulkan_CreateFontsTexture();
    }

    gpu_timers.frame = time_keeper.allocate_timer("frame");
    gpu_timers.draw = time_keeper.allocate_timer("draw");
    gpu_timers.tone_map = time_keeper.allocate_timer("tone_map");
    gpu_timers.ui = time_keeper.allocate_timer("ui");
    gpu_timers.compute_copy = time_keeper.allocate_timer("compute copy");
    gpu_timers.frame->nested_timers = {gpu_timers.draw, gpu_timers.tone_map, gpu_timers.ui, gpu_timers.compute_copy};
    time_keeper.initialize_timers();
    
    ui.frame_time_scope = gpu_timers.frame;
    ui.spp4 = &spp4;
}

void Renderer::shutdown() {
    wait_for_reference_renderer();
    VK_CHECK(vkDeviceWaitIdle(vk.device));

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    descriptor_heap.destroy();
    gpu_scene.point_lights.destroy();
    gpu_scene.directional_lights.destroy();
    gpu_scene.rect_lights.destroy();
    gpu_scene.lambertian_material_buffer.destroy();

    for (GPU_Mesh& mesh : gpu_meshes) {
        mesh.vertex_buffer.destroy();
        mesh.index_buffer.destroy();
    }
    gpu_meshes.clear();

    for (Vk_Image& image : gpu_scene.images_2d)
        image.destroy();

    gpu_scene.instance_info_buffer.destroy();
    gpu_scene.scene_info_buffer.destroy();

    apply_tone_mapping.destroy();
    copy_to_swapchain.destroy();
    release_resolution_dependent_resources();

    if (project_loaded) {
        patch_materials.destroy();
        direct_lighting.destroy();
        path_tracing.destroy();
    }

    vk_shutdown();
}

void Renderer::release_resolution_dependent_resources() {
    output_image.destroy();
    tonemapped_image.destroy();
}

void Renderer::restore_resolution_dependent_resources() {
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
            VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, descriptors.output_image);
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
            VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, descriptors.tonemapped_image);
    }

    // swapchain images
    for (size_t i = 0; i < vk.swapchain_info.images.size(); i++) {
        const uint32_t heap_offset = descriptors.swapchain_images + uint32_t(i * descriptor_heap.properties.imageDescriptorSize);
        descriptor_heap.write_image_descriptor(vk.swapchain_info.images[i], vk.surface_format.format,
            VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, heap_offset);
    }
}

void Renderer::load_project(const std::string& input_file) {
    wait_for_reference_renderer();
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

        std::vector<GPU_Types::Lambertian_Material> gpu_lambertian_materials(scene.materials.diffuse.size());
        for (auto[i, lambertian] : enumerate(scene.materials.diffuse)) {
            const RGB_Parameter& param = lambertian.reflectance;
            assert(param.eval_mode == EvaluationMode::value);
            if (param.value.is_constant) {
                gpu_lambertian_materials[i].r = param.value.constant.r;
                gpu_lambertian_materials[i].g = param.value.constant.g;
                gpu_lambertian_materials[i].b = param.value.constant.b;

                gpu_lambertian_materials[i].albedo_texture_index = -1;
                gpu_lambertian_materials[i].u_scale = 1.f;
                gpu_lambertian_materials[i].v_scale = 1.f;
            }
            else {
                gpu_lambertian_materials[i].r = 1.f;
                gpu_lambertian_materials[i].g = 1.f;
                gpu_lambertian_materials[i].b = 1.f;

                gpu_lambertian_materials[i].albedo_texture_index = param.value.texture.texture_index;
                gpu_lambertian_materials[i].u_scale = param.value.texture.u_scale;
                gpu_lambertian_materials[i].v_scale = param.value.texture.v_scale;
            }
        }

        if (!gpu_lambertian_materials.empty()) {
            VkDeviceSize size = gpu_lambertian_materials.size() * sizeof(GPU_Types::Lambertian_Material);
            gpu_scene.lambertian_material_buffer = vk_create_buffer(size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                gpu_lambertian_materials.data(), "lambertian_material_buffer");
        }
        descriptor_heap.write_buffer_descriptor(gpu_scene.lambertian_material_buffer.address_range(),
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.lambertian_materials);
    }

    // Textures
    {
        descriptors.images_2d = descriptor_heap.allocate_image_descriptor((uint32_t)gpu_scene.images_2d.size());
        for (auto [i, image] : enumerate(gpu_scene.images_2d)) {
            const uint32_t heap_offset = descriptors.images_2d + uint32_t(i * descriptor_heap.properties.imageDescriptorSize);
            descriptor_heap.write_image_descriptor(image.handle, image.format, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, heap_offset);
        }

        VkSamplerCreateInfo sampler_create_info{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        descriptor_heap.write_sampler_descriptor(sampler_create_info, descriptors.image_sampler);
    }

    // Geometry
    {
        descriptors.instance_infos = descriptor_heap.allocate_buffer_descriptor();
        descriptors.index_buffers = descriptor_heap.allocate_buffer_descriptor((uint32_t)gpu_meshes.size());
        descriptors.vertex_buffers = descriptor_heap.allocate_buffer_descriptor((uint32_t)gpu_meshes.size());

        descriptor_heap.write_buffer_descriptor(gpu_scene.instance_info_buffer.address_range(),
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.instance_infos);

        for (size_t i = 0; i < gpu_meshes.size(); i++) {
            const uint32_t element_offset = uint32_t(i * descriptor_heap.properties.bufferDescriptorSize);

            descriptor_heap.write_buffer_descriptor(gpu_meshes[i].index_buffer.address_range(),
                VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.index_buffers + element_offset);

            descriptor_heap.write_buffer_descriptor(gpu_meshes[i].vertex_buffer.address_range(),
                VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.vertex_buffers + element_offset);
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
        }
        if (!scene.lights.diffuse_rectangular_lights.empty()) {
            found_supported_lights = true;
            std::vector<GPU_Types::Rect_Light> lights(scene.lights.diffuse_rectangular_lights.size());
            for (auto [i, data] : enumerate(scene.lights.diffuse_rectangular_lights)) {
                lights[i].init(data);
            }
            gpu_scene.rect_lights = vk_create_buffer(lights.size() * sizeof(lights[0]),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                lights.data(), "rect_light_buffer");
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
            printf("No supported lights found. Added default directional light\n");
        }

        descriptor_heap.write_buffer_descriptor(gpu_scene.point_lights.address_range(), 
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.point_lights);

        descriptor_heap.write_buffer_descriptor(gpu_scene.directional_lights.address_range(), 
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.directional_lights);

        descriptor_heap.write_buffer_descriptor(gpu_scene.rect_lights.address_range(),
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.rect_lights);
    }

    // Scene info
    {
        GPU_Types::Scene_Info scene_info{};
        scene_info.point_light_count = (uint32_t)scene.lights.point_lights.size();
        scene_info.directional_light_count = (uint32_t)scene.lights.directional_lights.size();
        scene_info.rect_light_count = (uint32_t)scene.lights.diffuse_rectangular_lights.size();

        // Count default light if not lights are specified
        if (scene_info.point_light_count + scene_info.directional_light_count + scene_info.rect_light_count == 0) {
            scene_info.directional_light_count = 1;
        }

        gpu_scene.scene_info_buffer = vk_create_buffer(sizeof(GPU_Types::Scene_Info),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            &scene_info, "scene_info_buffer");

        descriptors.scene_info_buffer = descriptor_heap.allocate_buffer_descriptor();
        descriptor_heap.write_buffer_descriptor(gpu_scene.scene_info_buffer.address_range(),
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.scene_info_buffer);
    }

    std::vector<VkDescriptorSetAndBindingMappingEXT> global_heap_mappings;

    // Base resources
    global_heap_mappings.push_back(map_binding_to_heap_offset(
        BASE_SET_INDEX, 0, VK_SPIRV_RESOURCE_TYPE_SAMPLED_IMAGE_BIT_EXT,
        descriptors.images_2d, descriptors.image_descriptor_size
    ));
    global_heap_mappings.push_back(map_binding_to_heap_offset(
        BASE_SET_INDEX, 1, VK_SPIRV_RESOURCE_TYPE_SAMPLER_BIT_EXT,
        descriptors.image_sampler
    ));
    global_heap_mappings.push_back(map_binding_to_heap_offset(
        BASE_SET_INDEX, BASE_SET_BINDING_INSTANCE_INFO, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.instance_infos
    ));
    global_heap_mappings.push_back(map_binding_to_heap_offset(
        BASE_SET_INDEX, 3, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.index_buffers, descriptors.buffer_descriptor_size
    ));
    global_heap_mappings.push_back(map_binding_to_heap_offset(
        BASE_SET_INDEX, 4, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.vertex_buffers, descriptors.buffer_descriptor_size
    ));
    global_heap_mappings.push_back(map_binding_to_heap_offset(
        BASE_SET_INDEX, 5, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.scene_info_buffer, descriptors.buffer_descriptor_size
    ));

    // Materials
    global_heap_mappings.push_back(map_binding_to_heap_offset(
        MATERIAL_SET_INDEX, LAMBERTIAN_MATERIAL_BINDING, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_STORAGE_BUFFER_BIT_EXT,
        descriptors.lambertian_materials
    ));

    // Lights
    global_heap_mappings.push_back(map_binding_to_heap_offset(
        LIGHT_SET_INDEX, POINT_LIGHT_BINDING, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.point_lights
    ));
    global_heap_mappings.push_back(map_binding_to_heap_offset(
        LIGHT_SET_INDEX, DIRECTIONAL_LIGHT_BINDING, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.directional_lights
    ));
    global_heap_mappings.push_back(map_binding_to_heap_offset(
        LIGHT_SET_INDEX, RECT_LIGHT_BINDING, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.rect_lights
    ));

    patch_materials.create(descriptors);
    vk_execute(vk.command_pools[0], vk.queue, [this](VkCommandBuffer command_buffer) {
        descriptor_heap.bind(command_buffer);
        patch_materials.dispatch(command_buffer);
    });

    direct_lighting.create(descriptor_heap, descriptors, global_heap_mappings, scene, gpu_meshes);
    path_tracing.create(descriptor_heap, descriptors, global_heap_mappings, scene, gpu_meshes);

    project_loaded = true;
}

static double last_frame_time;

void Renderer::run_frame() {
    ui.reference_renderer_running = reference_renderer_running.load();

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
        accumulation_index = 0;
    }
    ui.camera_position = flying_camera.get_camera_pose().get_column(3);

    if (ui.reset_accumulation) {
        accumulation_index = 0;
    }
    if (ui_actions.reference_render_requested) {
        start_reference_renderer();
    }

    draw_frame();
    frame_index++;
    accumulation_index++;
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
    gpu_timers.frame->start();
    descriptor_heap.bind(vk.command_buffer);

    // Set per-frame parameters
    static_assert(sizeof(GPU_Types::Frame_Params) <= 256); // Vulkan guarantees 256 min limit
    GPU_Types::Frame_Params frame_params{};
    frame_params.frame_index = frame_index;
    frame_params.accumulation_index = accumulation_index;
    frame_params.swapchain_image_index = vk.swapchain_image_index;
    frame_params.spp4 = uint32_t(spp4);

    frame_params.camera_to_world = flying_camera.get_camera_pose();

    frame_params.viewport_size = { vk.surface_size.width, vk.surface_size.height };
    frame_params.tan_fovy_over_2 = std::tan(radians(scene.camera_fov_y / 2.f));
    frame_params.z_is_up = uint32_t(scene.z_is_up);

    VkPushDataInfoEXT push_data_info{ VK_STRUCTURE_TYPE_PUSH_DATA_INFO_EXT };
    push_data_info.data.address = &frame_params;
    push_data_info.data.size = sizeof(frame_params);
    vkCmdPushDataEXT(vk.command_buffer, &push_data_info);

    if (project_loaded) {
        draw_raytraced_image();
    }

    tone_mapping();

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, VK_IMAGE_LAYOUT_UNDEFINED,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL);

    copy_output_image_to_swapchain();

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL,
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    draw_imgui();

    vk_cmd_image_barrier(vk.command_buffer, vk.swapchain_info.images[vk.swapchain_image_index],
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);

    gpu_timers.frame->stop();
    vk_end_frame();
}

void Renderer::draw_raytraced_image() {
    VK_TIME_SCOPE(gpu_timers.draw);
    if (ui.rendering_algorithm == 0) {
        direct_lighting.dispatch();
    }
    else if (ui.rendering_algorithm == 1) {
        path_tracing.dispatch();
    }
}

void Renderer::tone_mapping()
{
    VK_TIME_SCOPE(gpu_timers.tone_map);
    apply_tone_mapping.dispatch();
}

void Renderer::draw_imgui()
{
    VK_TIME_SCOPE(gpu_timers.ui);

    ImGui::Render();

    VkRenderingAttachmentInfo color_attachment{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
    color_attachment.imageView = vk.swapchain_info.image_views[vk.swapchain_image_index];
    color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
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

void Renderer::copy_output_image_to_swapchain()
{
    VK_TIME_SCOPE(gpu_timers.compute_copy);
    copy_to_swapchain.dispatch();
}

void Renderer::start_reference_renderer()
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

void Renderer::do_run_reference_renderer(const Reference_Renderer_Config& reference_renderer_config, const Scene_Overrides& overrides)
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

void Renderer::wait_for_reference_renderer()
{
    if (reference_renderer_thread.joinable()) {
        reference_renderer_thread.join();
        reference_renderer_running.store(false);
    }
}
