#include "std.h"
#include "lib/common.h"
#include "gpu_scene.h"

#include "descriptor_heap.h"
#include "descriptors.h"
#include "lib/scene.h"

#include "shaders/shared.slang"

// TODO: temp structure. Use separate buffer per attribute.
struct GPU_Vertex {
    Vector3 position;
    Vector3 normal;
    Vector2 uv;
};

void GPU_Scene::load(const Scene& scene, Descriptor_Heap& descriptor_heap)
{
    // Meshes
    meshes.resize(scene.geometries.triangle_meshes.size());
    for (int i = 0; i < (int)scene.geometries.triangle_meshes.size(); i++) {
        const Triangle_Mesh& triangle_mesh = scene.geometries.triangle_meshes[i];
        GPU_Mesh& gpu_mesh = meshes[i];

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
        instance_info_buffer = vk_create_buffer(size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            instance_infos.data(), "instance_info_buffer");
    }

    // Materials.
    {
        images_2d.reserve(images_2d.size() + scene.texture_descriptors.size());
        for (const Texture_Descriptor& texture_desc : scene.texture_descriptors) {
            Vk_Image image = vk_load_texture(scene.get_resource_absolute_path(texture_desc.file_name));
            images_2d.push_back(image);
        }

        std::vector<GPU_Types::Lambertian_Material> gpu_lambertian_materials(scene.materials.diffuse.size());
        for (auto [i, lambertian] : enumerate(scene.materials.diffuse)) {
            const RGB_Parameter& param = lambertian.reflectance;
            ASSERT(param.eval_mode == EvaluationMode::value);
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
            lambertian_material_buffer = vk_create_buffer(size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                gpu_lambertian_materials.data(), "lambertian_material_buffer");
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
            point_lights = vk_create_buffer(lights.size() * sizeof(lights[0]),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                lights.data(), "point_light_buffer");
        }
        if (!scene.lights.directional_lights.empty()) {
            found_supported_lights = true;
            std::vector<GPU_Types::Directional_Light> lights(scene.lights.directional_lights.size());
            for (auto [i, data] : enumerate(scene.lights.directional_lights)) {
                lights[i].init(data);
            }
            directional_lights = vk_create_buffer(lights.size() * sizeof(lights[0]),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                lights.data(), "directional_light_buffer");
        }
        if (!scene.lights.diffuse_rectangular_lights.empty()) {
            found_supported_lights = true;
            std::vector<GPU_Types::Rect_Light> lights(scene.lights.diffuse_rectangular_lights.size());
            for (auto [i, data] : enumerate(scene.lights.diffuse_rectangular_lights)) {
                lights[i].init(data);
            }
            rect_lights = vk_create_buffer(lights.size() * sizeof(lights[0]),
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
            directional_lights = vk_create_buffer(sizeof(GPU_Types::Directional_Light),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                &gpu_light, "directional_light_buffer");
            printf("No supported lights found. Added default directional light\n");
        }
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

        scene_info_buffer = vk_create_buffer(sizeof(GPU_Types::Scene_Info),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            &scene_info, "scene_info_buffer");
    }

    descriptors.initialize(descriptor_heap);
    write_descriptors(descriptor_heap);
    loaded = true;
}

void GPU_Scene::destroy()
{
    loaded = false;
    point_lights.destroy();
    directional_lights.destroy();
    rect_lights.destroy();
    lambertian_material_buffer.destroy();

    for (Vk_Image& image : images_2d) {
        image.destroy();
    }
    images_2d.clear();

    for (GPU_Mesh& mesh : meshes) {
        mesh.vertex_buffer.destroy();
        mesh.index_buffer.destroy();
    }
    meshes.clear();

    instance_info_buffer.destroy();
    scene_info_buffer.destroy();
}

void GPU_Scene::write_descriptors(Descriptor_Heap& descriptor_heap)
{
    // Material descriptors
    descriptor_heap.write_buffer_descriptor(lambertian_material_buffer.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.lambertian_materials);

    // Image descriptors
    descriptors.images_2d = descriptor_heap.allocate_image_descriptor((uint32_t)images_2d.size() + Predefined_Texture_Count);
    for (auto [i, image] : enumerate(images_2d)) {
        const uint32_t heap_offset = descriptors.images_2d + uint32_t((i + Predefined_Texture_Count) * vk.descriptor_heap_properties.imageDescriptorSize);
        descriptor_heap.write_image_descriptor(image.handle, image.format, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, heap_offset);
    }

    // Sampler descriptors
    VkSamplerCreateInfo sampler_create_info{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    descriptor_heap.write_sampler_descriptor(sampler_create_info, descriptors.image_sampler);

    // Geometry descriptors
    descriptors.instance_infos = descriptor_heap.allocate_buffer_descriptor();
    descriptors.index_buffers = descriptor_heap.allocate_buffer_descriptor((uint32_t)meshes.size());
    descriptors.vertex_buffers = descriptor_heap.allocate_buffer_descriptor((uint32_t)meshes.size());

    descriptor_heap.write_buffer_descriptor(instance_info_buffer.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.instance_infos);

    for (size_t i = 0; i < meshes.size(); i++) {
        const uint32_t element_offset = uint32_t(i * vk.descriptor_heap_properties.bufferDescriptorSize);

        descriptor_heap.write_buffer_descriptor(meshes[i].index_buffer.address_range(),
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.index_buffers + element_offset);

        descriptor_heap.write_buffer_descriptor(meshes[i].vertex_buffer.address_range(),
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.vertex_buffers + element_offset);
    }

    // Light descriptors
    descriptor_heap.write_buffer_descriptor(point_lights.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.point_lights);

    descriptor_heap.write_buffer_descriptor(directional_lights.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.directional_lights);

    descriptor_heap.write_buffer_descriptor(rect_lights.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.rect_lights);

    // Scene info descriptors
    descriptors.scene_info_buffer = descriptor_heap.allocate_buffer_descriptor();
    descriptor_heap.write_buffer_descriptor(scene_info_buffer.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptors.scene_info_buffer);
}

std::vector<VkDescriptorSetAndBindingMappingEXT> GPU_Scene::get_scene_descriptor_mappings() const
{
    std::vector<VkDescriptorSetAndBindingMappingEXT> mappings;
    // Base resources
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_SET, SCENE_BINDING_IMAGES, VK_SPIRV_RESOURCE_TYPE_SAMPLED_IMAGE_BIT_EXT,
        descriptors.images_2d, vk_image_descriptor_size()
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_SET, SCENE_BINDING_SAMPLER, VK_SPIRV_RESOURCE_TYPE_SAMPLER_BIT_EXT,
        descriptors.image_sampler
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_SET, SCENE_BINDING_INSTANCE_INFO, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.instance_infos
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_SET, 3, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.index_buffers, vk_buffer_descriptor_size()
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_SET, 4, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.vertex_buffers, vk_buffer_descriptor_size()
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_SET, 5, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.scene_info_buffer, vk_buffer_descriptor_size()
    ));
    // Materials
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_MATERIAL_SET, SCENE_MATERIAL_BINDING_LAMBERTIAN, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_STORAGE_BUFFER_BIT_EXT,
        descriptors.lambertian_materials
    ));
    // Lights
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_LIGHT_SET, SCENE_LIGHT_BINDING_POINT, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.point_lights
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_LIGHT_SET, SCENE_LIGHT_BINDING_DIRECTIONAL, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.directional_lights
    ));
    mappings.push_back(map_binding_to_heap_offset(
        SCENE_LIGHT_SET, SCENE_LIGHT_BINDING_RECT, VK_SPIRV_RESOURCE_TYPE_READ_ONLY_STORAGE_BUFFER_BIT_EXT,
        descriptors.rect_lights
    ));
    return mappings;
}
