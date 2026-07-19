#include "std.h"
#include "lib/common.h"
#include "gpu_scene.h"

#include "descriptor_heap.h"
#include "descriptor_offsets.h"
#include "lib/scene.h"

#include "shaders/shared.slang"

// TODO: temp structure. Use separate buffer per attribute.
struct GPU_Vertex {
    Vector3 position;
    Vector3 normal;
    Vector2 uv;
};

void GPU_Scene::load(const Scene& scene, Descriptor_Heap& descriptor_heap, const Descriptor_Offsets& descriptor_offsets)
{
    // Meshes
    {
        uint64_t total_mesh_vertex_size = 0;
        uint64_t total_mesh_index_size = 0;
        uint64_t max_mesh_vertex_size = 0;
        uint64_t max_mesh_index_size = 0;
        for (const Triangle_Mesh& mesh : scene.geometries.triangle_meshes) {
            const uint64_t vertex_size = (uint64_t)mesh.vertices.size() * sizeof(GPU_Vertex);
            const uint64_t index_size = (uint64_t)mesh.indices.size() * sizeof(uint32_t);
            max_mesh_vertex_size = std::max(max_mesh_vertex_size, vertex_size);
            max_mesh_index_size = std::max(max_mesh_index_size, index_size);
            total_mesh_vertex_size += vertex_size;
            total_mesh_index_size += index_size;
        }

        const VkBufferUsageFlags vertex_usage_flags =
            VK_BUFFER_USAGE_TRANSFER_DST_BIT |
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
            VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
        mesh_vertex_data = vk_create_buffer(total_mesh_vertex_size, vertex_usage_flags, nullptr, "mesh_vertex_data");

        const VkBufferUsageFlags index_usage_flags =
            VK_BUFFER_USAGE_TRANSFER_DST_BIT |
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
            VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
        mesh_index_data = vk_create_buffer(total_mesh_index_size, index_usage_flags, nullptr, "mesh_index_data");

        Vk_Buffer vertex_scratch_buffer = vk_create_mapped_buffer(max_mesh_vertex_size,
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT, "vertex_scratch");
        Vk_Buffer index_scratch_buffer = vk_create_mapped_buffer(max_mesh_index_size,
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT, "index_scratch");

        const uint32_t mesh_count = (uint32_t)scene.geometries.triangle_meshes.size();
        std::vector<GPU_Types::Mesh_Info> mesh_infos(mesh_count);
        meshes.resize(mesh_count);

        uint32_t current_vertex = 0;
        uint32_t current_index = 0;

        for (uint32_t i = 0; i < mesh_count; i++) {
            mesh_infos[i].first_vertex = current_vertex;
            mesh_infos[i].first_index = current_index;

            // Initialize GPU_Mesh
            const Triangle_Mesh& mesh = scene.geometries.triangle_meshes[i];
            GPU_Mesh& gpu_mesh = meshes[i];
            gpu_mesh.first_vertex_offset = current_vertex * sizeof(GPU_Vertex);
            gpu_mesh.vertex_count = (uint32_t)mesh.vertices.size();
            gpu_mesh.first_index_offset = current_index * sizeof(uint32_t);
            gpu_mesh.index_count = (uint32_t)mesh.indices.size();
            // TODO: this is wrong! render objects list should not be indexed by geometry index. 
            // Will be fixed when gpu renderer will support Render_Objects (i.e. instancing).
            int area_light_index = i - int(scene.geometries.triangle_meshes.size() - scene.lights.diffuse_rectangular_lights.size());
            if (area_light_index >= 0)
                gpu_mesh.area_light_index = area_light_index;
            else
                gpu_mesh.material = scene.objects[i].material;

            // Copy vertices to subregion of global vertex/index buffer
            GPU_Vertex* p_vertex = vertex_scratch_buffer.get_mapped_data<GPU_Vertex>();
            for (size_t k = 0; k < gpu_mesh.vertex_count; k++, p_vertex++) {
                GPU_Vertex vertex{};
                vertex.position = mesh.vertices[k];
                if (!mesh.normals.empty()) {
                    vertex.normal = mesh.normals[k];
                }
                if (!mesh.uvs.empty()) {
                    vertex.uv = mesh.uvs[k];
                }
                *p_vertex = vertex;
            }
            memcpy(index_scratch_buffer.mapped_ptr, mesh.indices.data(), mesh.indices.size() * sizeof(uint32_t));

            const uint64_t vertex_data_size = gpu_mesh.vertex_count * sizeof(GPU_Vertex);
            const uint64_t index_data_size = gpu_mesh.index_count * sizeof(uint32_t);
            vk_execute(vk.command_pools[0], vk.queue,
                [this, &vertex_scratch_buffer, &index_scratch_buffer,
                current_vertex, current_index, vertex_data_size, index_data_size]
                (VkCommandBuffer command_buffer)
            {
                VkBufferCopy region{};
                region.srcOffset = 0;

                region.dstOffset = current_vertex * sizeof(GPU_Vertex);
                region.size = vertex_data_size;
                vkCmdCopyBuffer(command_buffer, vertex_scratch_buffer.handle, mesh_vertex_data.handle, 1, &region);

                region.dstOffset = current_index * sizeof(uint32_t);
                region.size = index_data_size;
                vkCmdCopyBuffer(command_buffer, index_scratch_buffer.handle, mesh_index_data.handle, 1, &region);
            });
            current_vertex += gpu_mesh.vertex_count;
            current_index += gpu_mesh.index_count;
        }
        vertex_scratch_buffer.destroy();
        index_scratch_buffer.destroy();

        mesh_info_buffer = vk_create_buffer(mesh_count * sizeof(GPU_Types::Mesh_Info),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, mesh_infos.data(), "mesh_infos");
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
        images.reserve(images.size() + scene.texture_descriptors.size());
        for (const Texture_Descriptor& texture_desc : scene.texture_descriptors) {
            Vk_Image image = vk_load_texture(scene.get_resource_absolute_path(texture_desc.file_name));
            images.push_back(image);
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

    // Acceleration structures
    {
        accelerator = create_intersection_accelerator(scene.objects, meshes, mesh_vertex_data.device_address, mesh_index_data.device_address);
    }
    write_descriptors(descriptor_heap, descriptor_offsets);
    loaded = true;
}

void GPU_Scene::destroy()
{
    loaded = false;
    point_lights.destroy();
    directional_lights.destroy();
    rect_lights.destroy();
    lambertian_material_buffer.destroy();

    for (Vk_Image& image : images) {
        image.destroy();
    }
    images.clear();

    accelerator.destroy();
    mesh_info_buffer.destroy();
    mesh_vertex_data.destroy();
    mesh_index_data.destroy();
    meshes.clear();
    instance_info_buffer.destroy();
    scene_info_buffer.destroy();
}

void GPU_Scene::write_descriptors(Descriptor_Heap& descriptor_heap, const Descriptor_Offsets& descriptor_offsets)
{
    // Image descriptors
    const uint32_t project_images_offset = descriptor_offsets.get_image_descriptor_offset(Image_Index::first_project_image);
    const uint32_t project_image_descriptors_size = uint32_t(images.size() * vk_image_descriptor_size());
    if (project_images_offset + project_image_descriptors_size > descriptor_heap.resource_reserved_region_offset) {
        error("Not enough descriptor heap space for image descriptors. Free space: %u bytes, image descriptors: %u bytes",
            descriptor_heap.resource_reserved_region_offset - project_images_offset,
            project_image_descriptors_size
        );
    }
    for (auto [i, image] : enumerate(images)) {
        const uint32_t image_index = static_cast<uint32_t>(Image_Index::first_project_image) + (uint32_t)i;
        const uint32_t heap_offset = descriptor_offsets.images + uint32_t(image_index * vk.descriptor_heap_properties.imageDescriptorSize);
        descriptor_heap.write_image_descriptor(image.handle, image.format, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, heap_offset);
    }

    // Sampler descriptors
    VkSamplerCreateInfo sampler_create_info{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    descriptor_heap.write_sampler_descriptor(sampler_create_info, descriptor_offsets.image_sampler);

    // Geometry descriptors
    descriptor_heap.write_buffer_descriptor(instance_info_buffer.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptor_offsets.instance_infos);

    descriptor_heap.write_buffer_descriptor(mesh_info_buffer.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptor_offsets.mesh_infos);

    descriptor_heap.write_buffer_descriptor(mesh_vertex_data.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptor_offsets.mesh_vertex_data);

    descriptor_heap.write_buffer_descriptor(mesh_index_data.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptor_offsets.mesh_index_data);

    // Material descriptors
    descriptor_heap.write_buffer_descriptor(lambertian_material_buffer.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptor_offsets.lambertian_materials);

    // Light descriptors
    descriptor_heap.write_buffer_descriptor(point_lights.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptor_offsets.point_lights);

    descriptor_heap.write_buffer_descriptor(directional_lights.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptor_offsets.directional_lights);

    descriptor_heap.write_buffer_descriptor(rect_lights.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptor_offsets.rect_lights);

    // Scene info descriptors
    descriptor_heap.write_buffer_descriptor(scene_info_buffer.address_range(),
        VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, descriptor_offsets.scene_info_buffer);

    // Intersection accelerator
    descriptor_heap.write_acceleration_structure_descriptor(
        accelerator.top_level_accel.device_address,
        descriptor_offsets.accelerator
    );
}
