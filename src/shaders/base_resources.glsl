#include "shared_main.h"

layout(set=BASE_SET_INDEX, binding=0) uniform texture2D images_2d[];
layout(set=BASE_SET_INDEX, binding=1) uniform sampler image_sampler; 

layout(std430, set=BASE_SET_INDEX, binding=BASE_SET_BINDING_INSTANCE_INFO)
readonly buffer Instance_Info_Buffer {
    Instance_Info instance_infos[];
};

layout(std430, set=BASE_SET_INDEX, binding=3)
readonly buffer Index_Buffer {
    uint indices[];
} index_buffers[];

struct Mesh_Vertex {
    float x, y, z;
    float nx, ny, nz;
    float u, v;
};

layout(std430, set=BASE_SET_INDEX, binding=4)
readonly buffer Vertex_Buffer {
    Mesh_Vertex vertices[];
} vertex_buffers[];
