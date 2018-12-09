#version 460
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"

struct Frag_In {
    vec3 normal;
    vec2 uv;
};

layout(location=0) in vec4 in_position;
layout(location=1) in vec3 in_normal;
layout(location=2) in vec2 in_uv;

layout(location = 0) out Frag_In frag_in;

layout(std140, binding=0) uniform Global_Uniform_Block {
    mat4x4 model_view_proj;
    mat4x4 model_view;
    mat4x4 view;
};

void main() {
    frag_in.normal = vec3(model_view * vec4(in_normal, 0.0));
    frag_in.uv = in_uv;
    gl_Position = model_view_proj * in_position;
}
