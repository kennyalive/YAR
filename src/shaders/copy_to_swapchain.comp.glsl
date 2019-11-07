#version 460
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"
#include "tone_mapping.glsl"

layout(local_size_x = 32, local_size_y = 32) in;

layout(push_constant) uniform Push_Constants {
    uvec2 viewport_size;
};

layout(set=0, binding=0) uniform sampler point_sampler;
layout(set=0, binding=1) uniform texture2D  output_image;
layout(set=0, binding=2, rgba8) uniform writeonly image2D swapchain_image;

void main() {
    ivec2 loc = ivec2(gl_GlobalInvocationID.xy);

    if (loc.x < viewport_size.x && loc.y < viewport_size.y) {
        float s = (gl_GlobalInvocationID.x + 0.5) / viewport_size.x;
        float t = (gl_GlobalInvocationID.y + 0.5) / viewport_size.y;
        vec4 color = textureLod(sampler2D(output_image, point_sampler), vec2(s, t), 0);

        // Tone mapping.
        const float LWhite = 2.f;
        const float one_over_LWhite_squared = 1.f / (LWhite*LWhite);
        float L = dot(color.rgb, vec3(0.2126, 0.7152, 0.0722));
        float L_ldr = tonemap_reinhard(L, one_over_LWhite_squared);
        float scale = L_ldr / L;
        color.xyz *= scale;

        imageStore(swapchain_image, loc, color);
    }
}
