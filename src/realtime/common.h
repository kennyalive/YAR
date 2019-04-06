#pragma once

#include "vk.h"
#include "../shaders/gpu_types.h"

struct GPU_Mesh {
    Vk_Buffer vertex_buffer;
    Vk_Buffer index_buffer;
    uint32_t model_vertex_count;
    uint32_t model_index_count;
    GPU_Types::Mesh_Material material;
};
