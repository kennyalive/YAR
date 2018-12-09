#version 460
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

struct Frag_In {
    vec3 normal;
    vec2 uv;
};

layout(location=0) in Frag_In frag_in;

layout(location = 0) out vec4 color_attachment0;

layout(push_constant) uniform Push_Constants {
    vec3    k_diffuse;
    float   padding0;
    vec3    k_specular;
    float   padding1;
};

layout(std140, binding=0) uniform Global_Uniform_Block {
    mat4x4 model_view_proj;
    mat4x4 model_view;
    mat4x4 view;
};

void main() {
    const vec3 light_dir = vec3(view * vec4(normalize(vec3(0, 1, 2)), 0));
    vec3 n = normalize(frag_in.normal);
    float i = max(0.f, dot(light_dir, n));
    vec3 color = k_diffuse * i;
    color_attachment0 = vec4(srgb_encode(color), 1);
}
