#pragma once

#include "lib/material.h"
#include "vk.h"

struct GPU_Mesh {
    Vk_Buffer vertex_buffer;
    Vk_Buffer index_buffer;
    int32_t model_vertex_count = 0;
    int32_t model_index_count = 0;
    Material_Handle material;
    int area_light_index = -1;
};
