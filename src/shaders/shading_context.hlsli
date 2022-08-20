#ifndef SHADING_CONTEXT_HLSL
#define SHADING_CONTEXT_HLSL

struct Shading_Context {
    float3 Wo; // outgoing direction
    float3 P; // shading point position in world coordinates
    float3 Ng; // geometric normal oriented to be in the same hemisphere as Wo.
    float3 N; // shading normal
    float2 UV; // surface UV parameterization
};

#endif
