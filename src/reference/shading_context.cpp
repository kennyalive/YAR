#include "std.h"
#include "lib/common.h"
#include "shading_context.h"

#include "bsdf.h"
#include "intersection.h"
#include "parameter_evaluation.h"
#include "scene_context.h"
#include "thread_context.h"

#include "lib/math.h"
#include "lib/scene_object.h"

// Shading normal adaptation algorithm as described in:
//  "The Iray Light Transport Simulation and Rendering System", Keller et al. 2017
// Returns true if shading normal was modified and false otherwise.
static bool adjust_shading_normal(const Vector3& wo, const Vector3& ng, Vector3* n) {
    // renderer convention: 'wo' direction is in the hemisphere of the geometric normal
    ASSERT(dot(wo, ng) >= 0.f);
    // renderer convention: shading normal is in the hemisphere of the geometric normal
    ASSERT(dot(*n, ng) >= 0.f);

    Vector3 R = reflect(wo, *n);

    // If reflected direction is above the geometric surface then
    // shading normal adaptation is not needed.
    float a = dot(R, ng);
    if (a >= 0.f)
        return false;

    // For almost tangential 'wo' we have catastrophic cancellation in 'wo + tangent' expression below.
    // For this configuration we know that the result will be close to geometric normal, so return it directly.
    const float cos_threshold = 0.0017453f; // cos(89.9 degrees)
    if (dot(wo, ng) < cos_threshold) {
        *n = ng;
        return true;
    }

    float b = dot(*n, ng);

    Vector3 tangent;
    if (b > 1e-4f) {
        float distance_to_surface_along_normal = std::abs(a) / b;
        tangent = R + distance_to_surface_along_normal * (*n);
        tangent.normalize();
    }
    else {
        // For small 'b' (especially when it's zero) it's numerically challenging to compute 'tangent' as we do above.
        // For this configuration shading normal is almost tangential, so use it as a tangent vector.
        tangent = *n;
    }

    // This epsilon pulls tangent vector up a bit, so it's not strictly parallel to the surface.
    // In the scenario of ideal specular reflection it helps to avoid self intersection.
    const float epsilon = 1e-4f;
    tangent += epsilon * ng;
    ASSERT(dot(tangent, ng) > 0.f);

    Vector3 new_n = (wo + tangent).normalized();
    ASSERT(dot(wo, new_n) >= 0.f);
    *n = new_n;
    return true;
}

void Shading_Context::initialize_local_geometry(Thread_Context& thread_ctx, const Ray& ray,
    const Differential_Rays* differential_rays, const Intersection& intersection)
{
    *this = Shading_Context{};

    wo = -ray.direction.normalized();

    // Geometry_Type-specific initialization.
    // The values should be calculated in the local coordinate system of the object.
    // The following fields should be initialized:
    //  * position
    //  * geometric_normal, normal
    //  * has_uv_parameterization, uv, dpdu, dpdv, dndu, dndv
    if (intersection.geometry_type == Geometry_Type::triangle_mesh) {
        init_from_triangle_mesh_intersection(intersection.triangle_intersection);
    }
    else {
        ASSERT(false);
    }

    // Transform geometry data to world space.
    {
        const Matrix3x4& object_to_world = intersection.scene_object->object_to_world_transform;
        position = transform_point (object_to_world, position);

        const Matrix3x4& object_to_world_normal = intersection.scene_object->object_to_world_normal_transform;

        normal = transform_vector(object_to_world_normal, normal);
        float length_of_scaled_normal;
        normal.normalize(&length_of_scaled_normal);

        geometric_normal = transform_vector(object_to_world_normal, geometric_normal);
        geometric_normal.normalize();

        dpdu = transform_vector(object_to_world, dpdu);
        dpdv = transform_vector(object_to_world, dpdv);

        dndu = transform_vector(object_to_world_normal, dndu);
        dndv = transform_vector(object_to_world_normal, dndv);
        // If normal had non-unit length after object_to_world transform due to scaling then in
        // addition to normal normalization we also have to scale normal derivatives by the same
        // magnitude to ensure they represent the same change in normal direction.
        float inv_length_of_scaled_normal = 1 / length_of_scaled_normal;
        dndu *= inv_length_of_scaled_normal;
        dndv *= inv_length_of_scaled_normal;
    }

    // Check rare case when shading normal is orthogonal to geometric normal.
    // This can happen with small triangles where computation of geometric normal
    // suffers from catastrophic cancellation. Use shading normal as geometric
    // normal in this scenario.
    if (dot(normal, geometric_normal) == 0.f) {
        geometric_normal = normal;
    }

    // Enforce renderer convention that direction of the incident ray (wo) is in
    // the hemisphere of the geometric normal.
    // Additionally, adjust_shading_normal() below ensures that 'wo' is in the
    // hemisphere of the shading normal.
    if (dot(geometric_normal, wo) < 0) {
        geometric_normal = -geometric_normal;
    }

    // Ensure that shading normal is in the hemisphere of the geometric normal.
    if (dot(normal, geometric_normal) < 0) {
        normal = -normal;
        dndu = -dndu;
        dndv = -dndv;
        original_shading_normal_was_flipped = true;
    }

    offset_ray_origin_in_both_directions(position, geometric_normal,
        &ray_origin_for_positive_normal_direction,
        &ray_origin_for_negative_normal_direction
    );

    if (differential_rays)
        calculate_dxdy_derivatives(*differential_rays);

    // Adjustment of shading normal invalidates dndu/dndv. For now it's not clear to which degree
    // it could be an issue (in most cases shading normals are left unchanged). Until further evidence
    // we assume that dndu/dndv is still a reasonable approximation.
    shading_normal_adjusted = adjust_shading_normal(wo, geometric_normal, &normal);
    normal_before_bump = normal;

    if (!has_uv_parameterization) {
        ASSERT(dpdu == Vector3_Zero);
        ASSERT(dpdv == Vector3_Zero);
        coordinate_system_from_vector(normal, &dpdu_shading, &dpdv_shading);
    }
    else if (std::abs(dot(normal, geometric_normal) - 1.f) < 1e-6f) {
        dpdu_shading = dpdu;
        dpdv_shading = dpdv;
    }
    else {
        dpdu_shading = project_vector_onto_plane_and_get_direction(dpdu, normal);
        dpdu_shading *= dpdu.length();

        dpdv_shading = project_vector_onto_plane_and_get_direction(dpdv, normal);
        dpdv_shading *= dpdv.length();
    }
    dpdu_shading_before_bump = dpdu_shading;
    dpdv_shading_before_bump = dpdv_shading;

    material = intersection.scene_object->material;
    area_light = intersection.scene_object->area_light;
    nested_dielectric = intersection.scene_object->participate_in_nested_dielectrics_tracking;
}

void Shading_Context::initialize_scattering(Thread_Context& thread_ctx, float* u_scattering_type)
{
    if (material == Null_Material) // TODO: do we need this? if yes, write a comment when it's needed
        return;

    delta_scattering_event = check_for_delta_scattering_event(thread_ctx, u_scattering_type, &delta_scattering);

    bsdf_layer_selection_probability = 1.f - delta_scattering.delta_layer_selection_probability;
    ASSERT(bsdf_layer_selection_probability >= 0.f && bsdf_layer_selection_probability <= 1.f);

    if (bsdf_layer_selection_probability != 0.f)
        bsdf = create_bsdf(thread_ctx, material);
}

void Shading_Context::init_from_triangle_mesh_intersection(const Triangle_Intersection& ti)
{
    Vector3 p[3];
    ti.mesh->get_positions(ti.triangle_index, p);
    position = barycentric_interpolate(p, ti.barycentrics);

    geometric_normal = cross(p[1] - p[0], p[2] - p[0]).normalized();
    if (ti.mesh->reverse_geometric_normal_orientation)
        geometric_normal = -geometric_normal;

    normal = geometric_normal;

    Vector3 n[3];
    if (!ti.mesh->normals.empty()) {
        ti.mesh->get_normals(ti.triangle_index, n);
        Vector3 interpolated_normal = barycentric_interpolate(n, ti.barycentrics);
        if (interpolated_normal.length_squared() != 0) {
            normal = interpolated_normal.normalized();
        }
    }

    if (!ti.mesh->uvs.empty()) {
        has_uv_parameterization = true;
        Vector2 uvs[3];
        ti.mesh->get_uvs(ti.triangle_index, uvs);
        uv = barycentric_interpolate(uvs, ti.barycentrics);

        float a[2][2] = {
            { uvs[1].u - uvs[0].u, uvs[1].v - uvs[0].v },
            { uvs[2].u - uvs[0].u, uvs[2].v - uvs[0].v }
        };

        Vector3 bp[2] = { p[1] - p[0], p[2] - p[0] };
        if (float det = solve_linear_system_2x2(a, bp, &dpdu, &dpdv); std::abs(det) < 1e-9f) {
            coordinate_system_from_vector(geometric_normal, &dpdu, &dpdv);
        }

        Vector3 bn[2] = { n[1] - n[0], n[2] - n[0] };
        if (float det = solve_linear_system_2x2(a, bn, &dndu, &dndv); std::abs(det) < 1e-9f) {
            // NOTE: this is a pbrt's approximation
            Vector3 dn = cross(bn[1], bn[0]);
            // NOTE: length/squared_lenght can be zero even if the vector is non-zero but with small components.
            // That's the reason to check here for a scalar length value compared to checking if vector is zero.
            if (dn.length_squared() == 0.f) {
                dndu = Vector3{};
                dndv = Vector3{};
            }
            else {
                coordinate_system_from_vector(dn, &dndu, &dndv);
            }
        }
    }
}

void Shading_Context::calculate_dxdy_derivatives(const Differential_Rays& differential_rays)
{
    if (!has_uv_parameterization)
        return;

    // NOTE: dpdx, dpdy, dwo_dx, dwo_dy derivatives do not depend on uv parameterization
    // and are computed from differential rays alone. We can rework the logic here and
    // to have them available even if uv derivatives are not available.
    //
    // Also it looks that computation of differential rays for specular reflection
    // and transmission fundamentally does not require uv parameterization. Currently
    // we rely on it to compute dndx/dndy using chain rule but the intuition that we can
    // compute directly since we know how normal changes when we change position,
    // (some form of dn/dp) and we also have dpdx/dpdy.

    has_dxdy_derivatives = true;

    // Position derivatives.
    float plane_d = -dot(geometric_normal, position);
    float tx = ray_plane_intersection(differential_rays.dx_ray, geometric_normal, plane_d);
    if (std::abs(tx) != Infinity) {
        Vector3 px = differential_rays.dx_ray.get_point(tx);
        dpdx = px - position;
    }
    float ty = ray_plane_intersection(differential_rays.dy_ray, geometric_normal, plane_d);
    if (std::abs(ty) != Infinity) {
        Vector3 py = differential_rays.dy_ray.get_point(ty);
        dpdy = py - position;
    }

    // Direction derivatives.
    dwo_dx = (-differential_rays.dx_ray.direction) - wo;
    dwo_dy = (-differential_rays.dy_ray.direction) - wo;

    // UV derivatives.
    calculate_uv_derivates();

    // When differential rays are tracked for a sequence of specular bounces they are becoming
    // progressively worse approximation for a pixel footprint and so the derived values.
    // Here's a sanity check that disables differential rays functionality for implausible values.
    //
    // NOTE: this code is mostly designed to prevent unstable numerical calculations (which triggers asserts)
    // but is not a mean to disable bad differential rays early enough.
    // Raytracer_Config::max_differential_ray_specular_bounces should be used instead.
    if (std::abs(dudx) > 1e9f ||
        std::abs(dvdx) > 1e9f ||
        std::abs(dudy) > 1e9f ||
        std::abs(dvdy) > 1e9f)
    {
        has_dxdy_derivatives = false;
        dpdx = dpdy = dwo_dx = dwo_dy = Vector3{};
        dudx = dvdx = dudy = dvdy = 0.f;
    }
}

void Shading_Context::calculate_uv_derivates()
{
    // We need to solve these two linear systems (PBRT 3, 10.1.1):
    //  dpdx = dpdu * dudx + dpdv * dvdx (3 equations)
    //  dpdy = dpdu * dudy + dpdv * dvdy (3 equations)

    // In a system of 3 linear equations with 2 unknown variables it's
    // possible that one equation is degenerate. Here we get rid of the
    // equation with the highest chance to be degenerate.
    int dim0, dim1;
    {
        Vector3 a = geometric_normal.abs();
        if (a.x > a.y && a.x > a.z) {
            dim0 = 1;
            dim1 = 2;
        }
        else if (a.y > a.z) {
            dim0 = 0;
            dim1 = 2;
        }
        else {
            dim0 = 0;
            dim1 = 1;
        }
    }

    float a[2][2] = {
        { dpdu[dim0], dpdv[dim0] },
        { dpdu[dim1], dpdv[dim1] }
    };
    float bx[2] = {
        dpdx[dim0],
        dpdx[dim1]
    };
    float by[2] = {
        dpdy[dim0],
        dpdy[dim1]
    };
    // If equation cannot be solved then derivatives stay initialized to zero.
    solve_linear_system_2x2(a, bx, &dudx, &dvdx);
    solve_linear_system_2x2(a, by, &dudy, &dvdy);
}

float Shading_Context::compute_texture_lod(int mip_count, const Vector2& uv_scale) const
{
    Vector2 dUVdx_scaled = Vector2(dudx, dvdx) * uv_scale;
    Vector2 dUVdy_scaled = Vector2(dudy, dvdy) * uv_scale;

    // To satisfy Nyquist limit the filter width should be twice as large as computed here.
    // This is achieved implicitly by using bilinear filtering to sample mip levels.
    /*
    float filter_width = std::max(
            std::max(std::abs(dUVdx_scaled.u), std::abs(dUVdx_scaled.v)),
            std::max(std::abs(dUVdy_scaled.u), std::abs(dUVdy_scaled.v)));
    */
    float filter_width = std::max(dUVdx_scaled.length(), dUVdy_scaled.length());

    return std::max(0.f, mip_count - 1 + log2(std::clamp(filter_width, 1e-6f, 1.0f)));
}

Differential_Rays Shading_Context::compute_differential_rays_for_specular_reflection(const Ray& reflected_ray) const
{
    ASSERT(has_dxdy_derivatives);
    const float dot_wo_n = dot(wo, normal);

    Ray dx_ray { reflected_ray.origin + dpdx };
    Vector3 dndx = dndu * dudx + dndv * dvdx;
    float d_wo_dot_n_dx = dot(dwo_dx, normal) + dot(wo, dndx);
    Vector3 dwi_dx = 2.f * (d_wo_dot_n_dx * normal + dot_wo_n * dndx) - dwo_dx;
    dx_ray.direction = (reflected_ray.direction + dwi_dx).normalized();

    Ray dy_ray { reflected_ray.origin + dpdy };
    Vector3 dndy = dndu * dudy + dndv * dvdy;
    float d_wo_dot_n_dy = dot(dwo_dy, normal) + dot(wo, dndy);
    Vector3 dwi_dy = 2.f * (d_wo_dot_n_dy * normal + dot_wo_n * dndy) - dwo_dy;
    dy_ray.direction = (reflected_ray.direction + dwi_dy).normalized();

    return Differential_Rays { dx_ray, dy_ray };
}

Differential_Rays Shading_Context::compute_differential_rays_for_specular_transmission(const Ray& transmitted_ray, float etaI_over_etaT) const
{
    ASSERT(has_dxdy_derivatives);
    const float eta = etaI_over_etaT;
    float cos_o = dot(wo, normal);
    ASSERT(cos_o > 0.f);
    float cos_t = -dot(transmitted_ray.direction, normal);
    ASSERT(cos_t > 0.f);
    float k1 = eta * eta * cos_o / cos_t;
    float k2 = eta * cos_o - cos_t;

    Ray dx_ray { transmitted_ray.origin + dpdx };
    Vector3 dndx = dndu * dudx + dndv * dvdx;
    float d_wo_dot_n_dx = dot(dwo_dx, normal) + dot(wo, dndx);
    float d_cos_t_dx = k1 * d_wo_dot_n_dx;
    Vector3 dwi_dx = -eta * dwo_dx + k2 * dndx + (eta * d_wo_dot_n_dx - d_cos_t_dx) * normal;
    dx_ray.direction = (transmitted_ray.direction + dwi_dx).normalized();

    Ray dy_ray { transmitted_ray.origin + dpdy };
    Vector3 dndy = dndu * dudy + dndv * dvdy;
    float d_wo_dot_n_dy = dot(dwo_dy, normal) + dot(wo, dndy);
    float d_cos_t_dy = k1 * d_wo_dot_n_dy;
    Vector3 dwi_dy = -eta * dwo_dy + k2 * dndy + (eta * d_wo_dot_n_dy - d_cos_t_dy) * normal;
    dy_ray.direction = (transmitted_ray.direction + dwi_dy).normalized();

    return Differential_Rays { dx_ray, dy_ray };
}

Vector3 Shading_Context::get_ray_origin_using_control_direction(const Vector3& hemisphere_direction) const
{
    if (dot(hemisphere_direction, geometric_normal) > 0.f)
        return ray_origin_for_positive_normal_direction;
    else
        return ray_origin_for_negative_normal_direction;
}

Vector3 Shading_Context::get_ray_origin_using_control_point(const Vector3& hemisphere_point) const
{
    Vector3 hemisphere_direction = hemisphere_point - position;
    return get_ray_origin_using_control_direction(hemisphere_direction);
}

void Shading_Context::apply_bump_map(const Scene_Context& scene_ctx, Float_Parameter bump_map)
{
    if (bump_map.eval_mode != EvaluationMode::none) {
        float height = evaluate_float_parameter(scene_ctx, *this, bump_map);

        Vector2 duvdx = Vector2(dudx, dvdx);
        Vector2 duvdy = Vector2(dudy, dvdy);

        float du = 0.5f * (std::abs(dudx) + std::abs(dudy));
        if (du == 0.f) du = 0.0005f;
        Vector2 uv_du = uv + Vector2(du, 0);
        float height_du = evaluate_float_parameter(scene_ctx, uv_du, duvdx, duvdy, bump_map);

        float dv = 0.5f * (std::abs(dvdx) + std::abs(dvdy));
        if (dv == 0.f) dv = 0.0005f;
        Vector2 uv_dv = uv + Vector2(0, dv);
        float height_dv = evaluate_float_parameter(scene_ctx, uv_dv, duvdx, duvdy, bump_map);

        // bump map offset is relative to the unmodified shading normal direction, as defined by the geometry
        Vector3 original_shading_normal = original_shading_normal_was_flipped ? -normal_before_bump : normal_before_bump;

        // Parameter evaluation takes into account uv scale and the returned height values are in
        // the scaled uv space. To have matching units du/dv should also be scaled the same way.
        float du_scale = 1.f;
        float dv_scale = 1.f;
        if (bump_map.eval_mode == EvaluationMode::value && !bump_map.value.is_constant) {
            du_scale = bump_map.value.texture.u_scale;
            dv_scale = bump_map.value.texture.v_scale;
        }

        dpdu_shading = dpdu_shading_before_bump + ((height_du - height) / (du * du_scale)) * original_shading_normal + height * (dndu * du_scale);
        dpdv_shading = dpdv_shading_before_bump + ((height_dv - height) / (dv * dv_scale)) * original_shading_normal + height * (dndv * dv_scale);
        normal = cross(dpdu_shading, dpdv_shading).normalized();

        // renderer convention: shading normals should be in the hemisphere of the geometric normal.
        if (dot(normal, geometric_normal) < 0) {
            normal = -normal;
        }

        shading_normal_adjusted = adjust_shading_normal(wo, geometric_normal, &normal);
        // NOTE: do not adjust dpdu_shading/dpdv_shading derivatives to match new normal orientation.
        // They have limited usage from this point on. dpdu_shading is used later to construct orthonormal
        // basis during BSDF initialization, but it should not be orthogonal to the normal.
    }
}

bool trace_ray(Thread_Context& thread_ctx, const Ray& ray, const Differential_Rays* differential_rays)
{
    Intersection isect;
    if (!thread_ctx.scene_context.kdtree_data.scene_kdtree.intersect(ray, isect)) {
        thread_ctx.shading_context = Shading_Context{};
        thread_ctx.shading_context.miss_ray = ray;
        return false;
    }
    thread_ctx.shading_context.initialize_local_geometry(thread_ctx, ray, differential_rays, isect);
    return true;
}
