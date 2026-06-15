#pragma once

#include "vk.h"

struct Scene;

struct GPU_Scene
{
    std::vector<Vk_Image> images_2d;
    Vk_Buffer instance_info_buffer;
    Vk_Buffer scene_info_buffer;
    Vk_Buffer point_lights;
    Vk_Buffer directional_lights;
    Vk_Buffer rect_lights;
    Vk_Buffer lambertian_material_buffer;

    void load(const Scene& scene);
    void destroy();
};
