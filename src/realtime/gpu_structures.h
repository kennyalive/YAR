#pragma once

#include "vk.h"

#include "lib/color.h"
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
