#pragma once

#include "lib/light.h"
#include "lib/material.h"
#include "lib/vector.h"

struct Intersection;
struct Render_Context;
struct Thread_Context;
class BSDF;

struct Shading_Point_Rays {
    Ray incident_ray;
    Ray auxilary_ray_dx_offset;
    Ray auxilary_ray_dy_offset;
};

// Contains all the necessary information to perform shading at the intersection point.
struct Shading_Context {
    Vector3 Wo; // outgoing direction
    Vector3 P; // shading point position in world coordinates
    Vector3 Ng; // geometric normal
    Vector3 N; // shading normal
    Vector2 UV; // surface UV parameterization

    // Position derivatives with respect to texture coordinates.
    Vector3 dPdu;
    Vector3 dPdv;

    // UV derivatives with respect to screen coordinates.
    // These values can be used to compute texture lod for mip-mapping.
    Vector2 dUVdx;
    Vector2 dUVdy;

    Light_Handle area_light;
    const BSDF* bsdf = nullptr;

    bool mirror_surface = false;
    ColorRGB mirror_reflectance;

    Shading_Context(
        const Render_Context& global_ctx,
        Thread_Context& thread_ctx,
        const Shading_Point_Rays& rays, const Intersection& intersection);

    float compute_texture_lod(int mip_count, const Vector2& uv_scale) const;
};
