#pragma once

#include "lib/light.h"
#include "lib/vector.h"

struct Intersection;
struct Triangle_Intersection;
struct Scene_Context;
struct Thread_Context;
struct BSDF;

// Contains all the necessary information to perform shading at the intersection point.
// We keep one instance of Shading_Context per thread.
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

    // This flag is mostly for debugging purposes to mark regions where shading normal adaptation was applied.
    // The shading normal adaptation modifies shading normal N and ensures that Wo is in the positive hemisphere.
    // Described in: "The Iray Light Transport Simulation and Rendering System", Keller et al. 2017
    bool shading_normal_adjusted = false;

    // If ray cast does not find a point to shade then this is the last traced ray (there could be more than one
    // ray cast if we traverse specular surfaces). This can be used to look up environment map.
    Ray miss_ray;

    Shading_Context() {}

    void initialize_from_intersection(const Scene_Context& scene_ctx, Thread_Context& thread_ctx,
        const Ray& ray, const Auxilary_Rays* auxilary_rays, const Intersection& intersection);

    float compute_texture_lod(int mip_count, const Vector2& uv_scale) const;

    // Transformation of directions between local coordinate system defined
    // by the normal and two tangent vectors and world space coordinate system.
    Vector3 local_to_world(const Vector3& local_direction) const;
    Vector3 world_to_local(const Vector3& world_direction) const;

private:
    // Copying is forbiden because we allow caching of shading context references (BSDF does this).
    // We still keep private default assignment for internal usage (convenient initialization with default values).
    Shading_Context& operator=(const Shading_Context&) = default;

    void init_from_triangle_mesh_intersection(const Triangle_Intersection& ti, Vector3* dPdu, Vector3* dPdv);
    void calculate_UV_derivates(const Auxilary_Rays& auxilary_rays, const Vector3& dPdu, const Vector3& dPdv);
};
