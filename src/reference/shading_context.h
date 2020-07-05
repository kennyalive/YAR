#pragma once

#include "lib/light.h"
#include "lib/vector.h"

struct Intersection;
struct Triangle_Intersection;
struct Render_Context;
struct Thread_Context;
struct BSDF;

struct Shading_Point_Rays {
    Ray incident_ray;
    Ray auxilary_ray_dx_offset;
    Ray auxilary_ray_dy_offset;
};

// Contains all the necessary information to perform shading at the intersection point.
struct Shading_Context {
    Vector3 P; // shading point position in world coordinates
    Vector3 N; // shading normal
    Vector3 Ng; // geometric normal
    Vector2 UV; // surface UV parameterization

    // UV derivatives with respect to screen coordinates.
    // These values can be used to compute texture lod for mip-mapping.
    Vector2 dUVdx;
    Vector2 dUVdy;

    Vector3 Wo; // outgoing direction

    Light_Handle area_light;
    const BSDF* bsdf = nullptr;
    bool mirror_surface = false;
    ColorRGB mirror_reflectance;

    Shading_Context(
        const Render_Context& global_ctx,
        Thread_Context& thread_ctx,
        const Shading_Point_Rays& rays, const Intersection& intersection);

    float compute_texture_lod(int mip_count, const Vector2& uv_scale) const;

private:
    void init_from_triangle_mesh_intersection(const Triangle_Intersection& ti, Vector3* dPdu, Vector3* dPdv);
    void calculate_UV_derivates(const Shading_Point_Rays& rays, const Vector3& dPdu, const Vector3& dPdv);
};
