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

    // Ray origin variables are computed by nudging shading 'position' along geometric normal.
    // The goal of having these positions is to avoid self-intersection problem when we spawn
    // new ray from the shading point.
    Vector3 ray_origin_for_positive_normal_direction;
    Vector3 ray_origin_for_negative_normal_direction;

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

    // Surface's BSDF. Can be null when 'delta_scattering_event' is true.
    // Some materials might be described by a BSDF function plus a delta
    // layer, in that case both BSDF and 'delta_scattering_event' are set.
    const BSDF* bsdf = nullptr;

    // When material has both BSDF and a delta layer, then this value is how
    // often we sample bsdf for path generation (as opposed to delta layer sampling).
    float bsdf_layer_selection_probability = 1.f;

    Specular_Scattering specular_scattering;
    bool delta_scattering_event = false;

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

    // The position returned by these functions is a shading point moved slightly away from the
    // surface to ensure that we don't have self-intersection problem. The hemisphere_direction
    // and hemisphere_point parameters allow to choose hemisphere into which we are going to
    // trace a new ray.
    Vector3 get_ray_origin_using_control_direction(const Vector3& hemisphere_direction) const;
    Vector3 get_ray_origin_using_control_point(const Vector3& hemisphere_point) const;

private:
    void init_from_triangle_mesh_intersection(const Triangle_Intersection& ti);
    void calculate_UV_derivates();
};

// Trace ray against scene geometry and initializes shading context for intersection point (if any).
// Returns true if intersection found, otherwise false.
bool trace_ray(Thread_Context& thread_ctx, const Ray& ray, const Differential_Rays* differential_rays);
