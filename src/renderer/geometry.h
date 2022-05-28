#pragma once

#include "lib/material.h"
#include "vk.h"

struct GPU_Mesh {
    Vk_Buffer vertex_buffer;
    Vk_Buffer index_buffer;
    uint32_t vertex_count = 0;
    uint32_t index_count = 0;
    Material_Handle material;
    int area_light_index = -1;
};
