#pragma once

#include "lib/light.h"
#include "lib/vector.h"
#include "specular_scattering.h"

struct Intersection;
struct Triangle_Intersection;
struct Thread_Context;
struct BSDF;

// Contains all the necessary information to perform shading at the intersection point.
// There is one instance of Shading_Context per thread.
struct Shading_Context {
    Vector3 wo; // outgoing direction
    Vector3 position; // shading point position in world coordinates
    Vector3 normal; // shading normal
    Vector2 uv; // surface uv parameterization
    Vector3 geometric_normal;

    // This renderer is built in such a way that in most cases it is not sensitive to the normal
    // orientation convention (internally we flip normal if necessary, so it is always in the
    // hemisphere of the outgoing ray). There are still cases (one case?) when we need to know
    // the original shading normal orientation (as defined by the source geometry). For example,
    // when nested dielectrics tracing is disabled or not applicable, then we use original shading
    // normal orientation to determine if we are inside or outside the object.
    // The following property allows to determine original shading normal orientation.
    bool original_shading_normal_was_flipped = false;

    Vector3 dpdu;
    Vector3 dpdv;
    Vector3 dndu;
    Vector3 dndv;

    // If true then dx/dy derivatives are available. Otherwise they are set to zero.
    bool has_dxdy_derivatives = false;
    // The dx/dy derivatives are defined with respect to an average distance between the samples (not pixels!).
    Vector3 dpdx;
    Vector3 dpdy;
    Vector3 dwo_dx;
    Vector3 dwo_dy;
    float dudx = 0.f;
    float dvdx = 0.f;
    float dudy = 0.f;
    float dvdy = 0.f;

    // Tangent vectors for shading geometry.
    // (tangent1, tangent2, N) triplet forms right-handed orthonormal coordinate system.
    Vector3 tangent1;
    Vector3 tangent2;

    Light_Handle area_light;

    // Scattering at intersection point. During light-surface interaction the scattering process can be
    // described either by the BSDF function or it can be a delta scattering event (in the latter case,
    // the directions of scattered light form a set of zero measure with respect to solid angle metric...
    // I'm just kidding, sorry. Not sorry, it's the simplest possible explanation!!!).
    // For compound materials that exhibit both finite and delta scattering the result of bsdf scattering
    // should be weighted by a Specular_Scattering::finite_scattering_weight to get correct estimator.
    const BSDF* bsdf = nullptr;
    Specular_Scattering specular_scattering;

    // This flag is mostly for debugging purposes to mark regions where shading normal adaptation was applied.
    // The shading normal adaptation modifies shading normal N and ensures that Wo is in the positive hemisphere.
    // Described in: "The Iray Light Transport Simulation and Rendering System", Keller et al. 2017
    bool shading_normal_adjusted = false;

    // If ray cast does not find a point to shade then this is the last traced ray (there could be more than one
    // ray cast if we traverse specular surfaces). This can be used to look up environment map.
    Ray miss_ray;

    Shading_Context() {}

    void initialize_from_intersection(Thread_Context& thread_ctx, const Ray& ray,
        const Differential_Rays* differential_rays, const Intersection& intersection);

    float compute_texture_lod(int mip_count, const Vector2& uv_scale) const;

    // Transformation of directions between local coordinate system defined
    // by the normal and two tangent vectors and world space coordinate system.
    Vector3 local_to_world(const Vector3& local_direction) const;
    Vector3 world_to_local(const Vector3& world_direction) const;

    Differential_Rays compute_differential_rays_for_specular_reflection(const Ray& reflected_ray) const;
    Differential_Rays compute_differential_rays_for_specular_transmission(const Ray& transmitted_ray, float etaI_over_etaT) const;

private:
    void init_from_triangle_mesh_intersection(const Triangle_Intersection& ti);
    void calculate_UV_derivates();
};

// Trace ray against scene geometry and initializes shading context for intersection point (if any).
// Returns true if intersection found, otherwise false.
bool trace_ray(Thread_Context& thread_ctx, const Ray& ray, const Differential_Rays* differential_rays);
