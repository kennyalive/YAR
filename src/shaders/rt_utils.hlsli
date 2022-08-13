#ifndef RT_UTILS_HLSL
#define RT_UTILS_HLSL

struct Ray_Payload {
    float3 rx_dir;
    float3 ry_dir;
    float3 color;
};

struct Shadow_Ray_Payload {
    float shadow_factor; // 0.0 or 1.0
};

#endif
