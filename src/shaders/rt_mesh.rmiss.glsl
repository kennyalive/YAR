#version 460
#extension GL_GOOGLE_include_directive : require
#extension GL_NV_ray_tracing : require

#include "common.glsl"
#include "rt_utils.glsl"

layout (location=0) rayPayloadInNV Ray_Payload payload;

void main() {
    payload.color = srgb_encode(vec3(0.f));
}
