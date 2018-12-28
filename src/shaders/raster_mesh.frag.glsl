#version 460
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

struct Frag_In {
    vec3 normal;
    vec3 pos;
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

const vec3 light_pos = vec3(0, 0, 1.5);
const vec3 light_intensity = vec3(2.82138848, 2.22179413, 2.12889075);

void main() {
    const vec3 light_pos_eye = vec3(view * vec4(light_pos, 1.0));
    const vec3 light_vec = light_pos_eye - frag_in.pos;
    const float light_dist_sq_inv = 1.f / dot(light_vec, light_vec);
    const vec3 light_dir = light_vec * sqrt(light_dist_sq_inv);

    vec3 n = normalize(frag_in.normal);

    vec3 L = (k_diffuse * Pi_Inv) * light_intensity * (light_dist_sq_inv * max(0, dot(n, light_dir)));

    color_attachment0 = vec4(srgb_encode(L), 1);
}
