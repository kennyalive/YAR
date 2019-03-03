#version 460
#extension GL_GOOGLE_include_directive : require
#extension GL_NV_ray_tracing : require

#include "common.glsl"
#include "rt_utils.glsl"

layout (location=1) rayPayloadInNV Shadow_Ray_Payload payload;

void main() {
    payload.shadow_factor = 0.0;
}
