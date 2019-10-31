#version 460
#extension GL_GOOGLE_include_directive : require

#define MATERIAL_SET_INDEX 0
#include "common.glsl"
#include "material.glsl"
#include "shared_image.h"

layout(local_size_x = 1) in;

void patch_black(inout int texture_index) {
    if (texture_index == -1)
        texture_index = Black_2D_Texture_Index;
    else
        texture_index += Predefined_Texture_Count;
}

void main() {
    for (int i = 0; i < lambertian_materials.length(); i++) {
        patch_black(lambertian_materials[i].albedo_texture_index);
    }
}

