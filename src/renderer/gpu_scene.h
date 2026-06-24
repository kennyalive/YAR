#pragma once

#include "lib/material.h"
#include "vk.h"

struct Scene;

struct GPU_Mesh {
    Vk_Buffer vertex_buffer;
    Vk_Buffer index_buffer;
    uint32_t vertex_count = 0;
    uint32_t index_count = 0;
    Material_Handle material;
    int area_light_index = -1;
};

struct GPU_Scene
{
    std::vector<Vk_Image> images_2d;
    std::vector<GPU_Mesh> meshes;
    Vk_Buffer instance_info_buffer;
    Vk_Buffer scene_info_buffer;
    Vk_Buffer point_lights;
    Vk_Buffer directional_lights;
    Vk_Buffer rect_lights;
    Vk_Buffer lambertian_material_buffer;

    void load(const Scene& scene);
    void destroy();
};
