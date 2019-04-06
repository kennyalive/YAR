#pragma once

#include "vk.h"

#include "lib/color.h"
#include "lib/io.h"
#include "lib/vector.h"

struct GPU_Mesh_Material {
    ColorRGB    k_diffuse;
    float       pad0;
    ColorRGB    k_specular;
    float       pad1;
};

struct GPU_Mesh {
    Vk_Buffer                   vertex_buffer;
    Vk_Buffer                   index_buffer;
    uint32_t                    model_vertex_count;
    uint32_t                    model_index_count;
    GPU_Mesh_Material           material;
};

struct GPU_Point_Light {
    Vector3     position;
    float       pad0;
    ColorRGB    intensity;
    float       pad1;
};

struct GPU_Diffuse_Rectangular_Light {
    Matrix3x4 light_to_world;
    ColorRGB emitted_radiance;
    float pad0;
    Vector2 size;
    float area;
    int shadow_ray_count;

    void init(const RGB_Diffuse_Rectangular_Light_Data& data) {
        light_to_world = data.light_to_world_transform;
        emitted_radiance = data.emitted_radiance;
        pad0 = 0.f;
        size = data.size;
        area = data.size.x * data.size.y;
        shadow_ray_count = data.shadow_ray_count;
    }
};
