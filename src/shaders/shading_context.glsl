#ifndef SHADING_CONTEXT_GLSL
#define SHADING_CONTEXT_GLSL

struct Shading_Context
{
    vec3 Wo; // outgoing direction
    vec3 P; // shading point position in world coordinates
    vec3 Ng; // geometric normal, oriented to be in the same hemisphere as Wo.
    vec3 N; // shading normal
    vec2 UV; // surface parameterization value
};

#endif // SHADING_CONTEXT_GLSL
