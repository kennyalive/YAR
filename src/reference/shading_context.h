#pragma once

#include "lib/light.h"
#include "lib/material.h"
#include "lib/vector.h"
#include "delta_scattering.h"

struct Intersection;
struct Triangle_Intersection;
struct Thread_Context;
struct BSDF;
struct Scene_Context;

// Contains all the necessary information to perform shading at the intersection point.
// There is one instance of Shading_Context per thread.
struct Shading_Context {
    Vector3 wo; // outgoing direction
    Vector3 position; // shading point position in world coordinates
    Vector3 geometric_normal;

    Vector3 normal_before_bump; // shading normal before bump/normal mapping
    Vector3 normal; // shading normal

    // Surface UV parameterization.
    bool has_uv_parameterization = false;
    Vector2 uv;

    // Position derivatives of the original geometry
    Vector3 dpdu;
    Vector3 dpdv;
    // Position derivatives adjusted to perturbed surface before bump/normal mapping
    Vector3 dpdu_shading_before_bump;
    Vector3 dpdv_shading_before_bump;
    // Position derivatives adjusted to perturbed surface
    Vector3 dpdu_shading;
    Vector3 dpdv_shading;

    // Shading normal derivatives.
    // NOTE: the renderer currently supports only the geometry types that have zero derivatives
    // of geometric normals. When a new geometry type is introduced we need to revisit whether
    // to keep both shading and geometric derivatives.
    Vector3 dndu;
    Vector3 dndv;

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

    // dx/dy derivatives. They are defined relative to the average distance between samples (not pixels!).
    bool has_dxdy_derivatives = false;
    Vector3 dpdx;
    Vector3 dpdy;
    Vector3 dwo_dx;
    Vector3 dwo_dy;
    float dudx = 0.f;
    float dvdx = 0.f;
    float dudy = 0.f;
    float dvdy = 0.f;

    Material_Handle material;
    Light_Handle area_light;

    bool nested_dielectric = false;

    // Surface's BSDF. Can be null when 'delta_scattering_event' is true.
    // Some materials might be described by a BSDF function plus a delta
    // layer, in that case both BSDF and 'delta_scattering_event' are set.
    const BSDF* bsdf = nullptr;

    // When material has both BSDF and a delta layer, then this value is how
    // often we sample bsdf for path generation (as opposed to delta layer sampling).
    float bsdf_layer_selection_probability = 1.f;

    Delta_Scattering delta_scattering;
    bool delta_scattering_event = false;

    // This flag is mostly for debugging purposes to mark regions where shading normal adaptation was applied.
    // The shading normal adaptation modifies shading normal N and ensures that Wo is in the positive hemisphere.
    // Described in: "The Iray Light Transport Simulation and Rendering System", Keller et al. 2017
    bool shading_normal_adjusted = false;

    // If ray cast does not find a point to shade then this is the last traced ray (there could be more than one
    // ray cast if we traverse specular surfaces). This can be used to look up environment map.
    Ray miss_ray;

    Shading_Context() {}

    void initialize_local_geometry(Thread_Context& thread_ctx, const Ray& ray,
        const Differential_Rays* differential_rays, const Intersection& intersection);

    // u_scattering_type is in/out parameter. When it's used by the delta scattering pipeline,
    // it can be re-normalized for subsequent usage in the bsdf pipeline.
    // Currently we have one case that does re-normalization (pbrt uber material).
    // If, for some reason, it won't be possible to re-normalize u_scattering_type in the new
    // scenarios, then we have to use separate random variables for delta scattering and bsdf pipelines.
    void initialize_scattering(Thread_Context& thread_ctx, float* u_scattering_type);

    float compute_texture_lod(int mip_count, const Vector2& uv_scale) const;

    Differential_Rays compute_differential_rays_for_specular_reflection(const Ray& reflected_ray) const;
    Differential_Rays compute_differential_rays_for_specular_transmission(const Ray& transmitted_ray, float etaI_over_etaT) const;

    // The position returned by these functions is a shading point moved slightly away from the
    // surface to ensure that we don't have self-intersection problem. The hemisphere_direction
    // and hemisphere_point parameters allow to choose hemisphere into which we are going to
    // trace a new ray.
    Vector3 get_ray_origin_using_control_direction(const Vector3& hemisphere_direction) const;
    Vector3 get_ray_origin_using_control_point(const Vector3& hemisphere_point) const;

    void apply_bump_map(const Scene_Context& scene_ctx, Float_Parameter bump_map);

private:
    void init_from_triangle_mesh_intersection(const Triangle_Intersection& ti);

    void calculate_dxdy_derivatives(const Differential_Rays& differential_rays);
    void calculate_uv_derivates();
};

// Trace ray against scene geometry and initializes shading context for intersection point (if any).
// Returns true if intersection found, otherwise false.
bool trace_ray(Thread_Context& thread_ctx, const Ray& ray, const Differential_Rays* differential_rays);
