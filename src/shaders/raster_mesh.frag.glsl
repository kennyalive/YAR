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

void main() {
    vec3 n = normalize(frag_in.normal);
    float i = max(0, n.z);
    vec3 color = k_diffuse * i;
    color_attachment0 = vec4(srgb_encode(color), 1);
}
