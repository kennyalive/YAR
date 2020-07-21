#pragma once

#include "lib/light.h"
#include "lib/vector.h"

struct Intersection;
struct Triangle_Intersection;
struct Scene_Context;
struct Thread_Context;
struct BSDF;

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

    // UV derivatives with respect to screen coordinates.
    // These values can be used to compute texture lod for mip-mapping.
    Vector2 dUVdx;
    Vector2 dUVdy;

    // Tangent vectors for shading geometry.
    // (tangent1, tangent2, N) triplet forms right-handed orthonormal coordinate system.
    Vector3 tangent1;
    Vector3 tangent2;

    Light_Handle area_light;
    const BSDF* bsdf = nullptr;
    bool mirror_surface = false;
    ColorRGB mirror_reflectance;

    Shading_Context(
        const Scene_Context& global_ctx,
        Thread_Context& thread_ctx,
        const Shading_Point_Rays& rays, const Intersection& intersection);

    float compute_texture_lod(int mip_count, const Vector2& uv_scale) const;

    // Transforms direction from the local coordinate system defined
    // by the normal and two tangent vectors to world space direction.
    Vector3 local_to_world(const Vector3& local_direction) const;

private:
    void init_from_triangle_mesh_intersection(const Triangle_Intersection& ti, Vector3* dPdu, Vector3* dPdv);
    void calculate_UV_derivates(const Shading_Point_Rays& rays, const Vector3& dPdu, const Vector3& dPdv);
};
