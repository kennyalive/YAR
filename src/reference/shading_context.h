#pragma once

#include "lib/light.h"
#include "lib/vector.h"

struct Intersection;
struct Triangle_Intersection;
struct Scene_Context;
struct Thread_Context;
struct BSDF;

enum class Specular_Surface_Type {
    none,
    perfect_reflector,
    perfect_refractor
};

struct Specular_Surface_Params {
    Specular_Surface_Type type = Specular_Surface_Type::none;
    ColorRGB reflectance_coeff;
    ColorRGB transmission_coeff;
    float etaI_over_etaT = 1.f; // relative index of refracton, incident side relative to transmitted side
};

// Contains all the necessary information to perform shading at the intersection point.
// We keep one instance of Shading_Context per thread.
struct Shading_Context {
    Vector3 wo; // outgoing direction

    Vector3 position; // shading point position in world coordinates
    Vector3 geometric_normal;
    Vector3 normal; // shading normal
    Vector2 uv; // surface uv parameterization

    Vector3 dpdu;
    Vector3 dpdv;
    Vector3 dndu;
    Vector3 dndv;

    // The dx/dy derivatives below (position and uv) measure the change with respect to
    // an average distance between the samples (not pixels!).
    //
    Vector3 dpdx;
    Vector3 dpdy;
    // The uv derivatices are used to compute texture lod for mip-mapping.
    float dudx = 0.f;
    float dvdx = 0.f;
    float dudy = 0.f;
    float dvdy = 0.f;

    // Tangent vectors for shading geometry.
    // (tangent1, tangent2, N) triplet forms right-handed orthonormal coordinate system.
    Vector3 tangent1;
    Vector3 tangent2;

    Light_Handle area_light;

    const BSDF* bsdf = nullptr;

    // specular_attenuation defines how scattering on specular surfaces scales radiance.
    // More details:
    // One important design decision is that trace_ray() function guarantees that resulted
    // shading point lies on the surface with a finite bsdf. If specific raycast ends on a
    // specular surface then trace_ray() propagates the ray further until it reaches
    // non-specular surface or leaves the scene. The specular_attenuation parameter tracks
    // how bounces through specular surfaces (if any) affect radiance value. The main motivation
    // behind this design is that it allows to keep standard definition of bsdf without the need
    // to extend it to handle delta surfaces. All special cases related to delta surfaces
    // migrate to ray casting routine.
    ColorRGB specular_attenuation = Color_White;

    // This flag is mostly for debugging purposes to mark regions where shading normal adaptation was applied.
    // The shading normal adaptation modifies shading normal N and ensures that Wo is in the positive hemisphere.
    // Described in: "The Iray Light Transport Simulation and Rendering System", Keller et al. 2017
    bool shading_normal_adjusted = false;

    // If ray cast does not find a point to shade then this is the last traced ray (there could be more than one
    // ray cast if we traverse specular surfaces). This can be used to look up environment map.
    Ray miss_ray;

    Shading_Context() {}

    void initialize_from_intersection(const Scene_Context& scene_ctx, Thread_Context& thread_ctx,
        const Ray& ray, const Auxilary_Rays* auxilary_rays, const Intersection& intersection, Specular_Surface_Params* specular_surface_params);

    float compute_texture_lod(int mip_count, const Vector2& uv_scale) const;

    // Transformation of directions between local coordinate system defined
    // by the normal and two tangent vectors and world space coordinate system.
    Vector3 local_to_world(const Vector3& local_direction) const;
    Vector3 world_to_local(const Vector3& world_direction) const;

private:
    // Copying is forbiden because we allow caching of shading context references (BSDF does this).
    // We still keep private default assignment for internal usage (convenient initialization with default values).
    Shading_Context& operator=(const Shading_Context&) = default;

    void init_from_triangle_mesh_intersection(const Triangle_Intersection& ti);
    void calculate_UV_derivates(const Auxilary_Rays& auxilary_rays);
};
