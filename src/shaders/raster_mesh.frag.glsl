#version 460
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

struct Frag_In {
    vec3 normal;
    vec3 pos;
    vec2 uv;
};

struct Point_Light {
    vec3 position;
    vec3 intensity;
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
    Point_Light point_lights[8];
    int point_light_count;
};

void main() {
    vec3 n = normalize(frag_in.normal);
    vec3 L = vec3(0);

    for (int i = 0; i < point_light_count; i++) {
        vec3 light_pos_eye = vec3(view * vec4(point_lights[i].position, 1.0));
        vec3 light_vec = light_pos_eye - frag_in.pos;
        float light_dist_sq_inv = 1.f / dot(light_vec, light_vec);
        vec3 light_dir = light_vec * sqrt(light_dist_sq_inv);

        L += (k_diffuse * Pi_Inv) * point_lights[i].intensity * (light_dist_sq_inv * max(0, dot(n, light_dir)));
    }

    color_attachment0 = vec4(srgb_encode(L), 1);
}
