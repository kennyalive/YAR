#include "std.h"
#include "lib/common.h"
#include "shading_context.h"

#include "bsdf.h"
#include "context.h"
#include "intersection.h"
#include "parameter_evaluation.h"

#include "lib/math.h"
#include "lib/scene_object.h"

// Shading normal adaptation algorithm as described in:
//  "The Iray Light Transport Simulation and Rendering System", Keller et al. 2017
// Returns true if shading normal was modified and false otherwise.
static bool adjust_shading_normal(const Vector3& wo, const Vector3& ng, Vector3* n) {
    // check renderer convention: shading frame is oriented in such way that Wo is in the positive hemisphere
    ASSERT(dot(wo, ng) >= 0.f);

    Vector3 R = reflect(wo, *n);

    // If reflected direction is above the geometric surface then
    // shading normal adaptation is not needed.
    float a = dot(R, ng);
    if (a >= 0.f)
        return false;

    // For almost tangential Wo we have catastrophic cancellation in 'Wo + tangent' expression below.
    // For this situation we know that the result will be close to geometric normal, so return it directly.
    if (dot(wo, ng) < 1e-3f) {
        *n = ng;
        return true;
    }

    float b = dot(*n, ng);
    ASSERT(b > 0.f);

    // This epsilon pulls tangent vector up a bit, so it's not strictly parallel to the surface.
    // In the scenario of ideal specular reflection it helps to avoid self intersection.
    float epsilon = 1e-4f;

    float distance_to_surface_along_normal = std::abs(a)/b;
    Vector3 tangent = R + (distance_to_surface_along_normal + epsilon) * (*n);
    tangent.normalize();

    // Check that tangent vector is either above the surface or only a little bit below it.
    // The second scenario is still possible in rare situations due to finite FP precision when we have
    // extreme orientation of the shading normal N relative to the geometric normal Ng (almost orthogonal).
    // This is not a problem per se, because tangent vector only provides characteristic orientation.
    ASSERT(dot(tangent, ng) > -1e-5f);

    Vector3 new_N = (wo + tangent).normalized();
    ASSERT(dot(wo, new_N) >= 0.f);
    *n = new_N;
    return true;
}

void Shading_Context::initialize_from_intersection(
    const Scene_Context& global_ctx,
    Thread_Context& thread_ctx,
    const Ray& ray, const Auxilary_Rays* auxilary_rays, const Intersection& intersection)
{
    *this = Shading_Context{};

    wo = -ray.direction;

    // Geometry_Type-specific initialization.
    //
    // The following fields should be initialized: P, N, Ng, UV, dPdu, dPdv, dNdu, dNdv.
    // The values should be calculated in the local coordinate system of the object.
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
        normal = transform_vector(object_to_world, normal);
        geometric_normal = transform_vector(object_to_world, geometric_normal);
        dpdu = transform_vector(object_to_world, dpdu);
        dpdv = transform_vector(object_to_world, dpdv);
        dndu = transform_vector(object_to_world, dndu);
        dndv = transform_vector(object_to_world, dndv);
    }

    // Trying to avoid self-shadowing.
    const bool transmission_event = intersection.scene_object->material.type == Material_Type::glass; // TODO: we need more general way to do this
    position = offset_ray_origin(position, transmission_event ? -geometric_normal : geometric_normal);

    if (auxilary_rays)
        calculate_UV_derivates(*auxilary_rays);

    // NOTE: adjustment of shading normal invalidates dndu/dndv. For now it's not clear to which degree
    // it could be an issue (in most cases shading normals are left unchanged). Until further evidence,
    // we assume that dndu/dndv is still a reasonable approximation.
    shading_normal_adjusted = adjust_shading_normal(wo, geometric_normal, &normal);

    if (dpdu != Vector3_Zero) {
        tangent2 = cross(normal, dpdu).normalized();
        tangent1 = cross(tangent2, normal);
    }
    else {
        ASSERT(dpdv == Vector3_Zero); // consistency check, expect both derivatives are zero
        coordinate_system_from_vector(normal, &tangent1, &tangent2);
    }

    area_light = intersection.scene_object->area_light;

    // Init scattering information.
    if (intersection.scene_object->material != Null_Material) {
        Material_Handle mtl = intersection.scene_object->material;
        if (mtl.type == Material_Type::mirror) {
            specular_scattering_type = Specular_Scattering_Type::reflection;
            const Mirror_Material& params = global_ctx.materials.mirror[mtl.index];
            specular_reflectance_coeff = evaluate_rgb_parameter(global_ctx, *this, params.reflectance);
        }
        else if (mtl.type == Material_Type::glass) {
            specular_scattering_type = Specular_Scattering_Type::transmission;
            const Glass_Material& params = global_ctx.materials.glass[mtl.index];
            specular_reflectance_coeff = evaluate_rgb_parameter(global_ctx, *this, params.reflectance);
            specular_transmission_coeff = evaluate_rgb_parameter(global_ctx, *this, params.transmittance);
            float dielectric_ior = evaluate_float_parameter(global_ctx, *this, params.index_of_refraction);

            if (thread_ctx.current_dielectric_material == Null_Material) { // dielectric enter event
                thread_ctx.current_dielectric_material = mtl;
                specular_etaI_over_etaT = 1.f / dielectric_ior;
            }
            else {  // dielectric exit event
                ASSERT(thread_ctx.current_dielectric_material == mtl);
                thread_ctx.current_dielectric_material = Null_Material;
                specular_etaI_over_etaT = dielectric_ior / 1.f;
            }
        }
        else {
            bsdf = create_bsdf(global_ctx, thread_ctx, *this, intersection.scene_object->material);
        }
    }
}

void Shading_Context::init_from_triangle_mesh_intersection(const Triangle_Intersection& ti) {
    position = ti.mesh->get_position(ti.triangle_index, ti.b1, ti.b2);

    Vector3 p0, p1, p2;
    ti.mesh->get_triangle(ti.triangle_index, p0, p1, p2);
    geometric_normal = cross(p1 - p0, p2 - p0).normalized();
    geometric_normal = dot(geometric_normal, wo) < 0 ? -geometric_normal : geometric_normal;

    normal = ti.mesh->get_normal(ti.triangle_index, ti.b1, ti.b2);
    const bool flip_shading_normal = dot(normal, geometric_normal) < 0;
    normal = flip_shading_normal ? -normal : normal;

    uv = ti.mesh->get_uv(ti.triangle_index, ti.b1, ti.b2);

    Vector2 uvs[3];
    ti.mesh->get_uvs(ti.triangle_index, uvs);
    float a[2][2] = {
        { uvs[1].u - uvs[0].u, uvs[1].v - uvs[0].v },
        { uvs[2].u - uvs[0].u, uvs[2].v - uvs[0].v }
    };

    // dpdu/dpdv
    {
        Vector3 b[2] = {
            p1 - p0,
            p2 - p0
        };
        // If equation cannot be solved then dpdu/dpdv stay initialized to zero.
        solve_linear_system_2x2(a, b, &dpdu, &dpdv);
    }
    // dndu/dndv
    {
        Vector3 normals[3];
        ti.mesh->get_normals(ti.triangle_index, normals);
        if (flip_shading_normal) {
            normals[0] = -normals[0];
            normals[1] = -normals[1];
            normals[2] = -normals[2];
        }
        Vector3 b[2] = {
            normals[1] - normals[0],
            normals[2] - normals[0]
        };
        // If equation cannot be solved then dndu/dndv stay initialized to zero.
        solve_linear_system_2x2(a, b, &dndu, &dndv);
    }
}

void Shading_Context::calculate_UV_derivates(const Auxilary_Rays& auxilary_rays) {
    // Compute position derivatives with respect to screen coordinates using auxilary offset rays.
    float plane_d = -dot(normal, position);
    float tx = ray_plane_intersection(auxilary_rays.ray_dx_offset, normal, plane_d);
    float ty = ray_plane_intersection(auxilary_rays.ray_dy_offset, normal, plane_d);

    Vector3 px = auxilary_rays.ray_dx_offset.get_point(tx);
    Vector3 py = auxilary_rays.ray_dy_offset.get_point(ty);
    dpdx = px - position;
    dpdy = py - position;

    // Compute uv derivatives with respect to screen coordinates (PBRT, 10.1.1).
    //
    // We need to solve these two linear systems:
    //  dPdx = dPdu * dUdx + dPdv * dVdx (3 equations)
    //  dPdy = dPdu * dUdy + dPdv * dVdy (3 equations)

    // In a system of 3 linear equations with 2 unknown variables it's
    // possible that one equation is degenerate. Here we get rid of equation
    // with the highest chance to be degenerate.
    int dim0 = 0, dim1 = 1;
    {
        Vector3 a = normal.abs();
        if (a.x > a.y && a.x > a.z) {
            dim0 = 1;
            dim1 = 2;
        }
        else if (a.y > a.z) {
            dim0 = 0;
            dim1 = 2;
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
    // If equation cannot be solved then derivative stay initialized to zero.
    solve_linear_system_2x2(a, bx, &dudx, &dvdx);
    solve_linear_system_2x2(a, by, &dudy, &dvdy);
}

float Shading_Context::compute_texture_lod(int mip_count, const Vector2& uv_scale) const {
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

Vector3 Shading_Context::local_to_world(const Vector3& local_direction) const {
    return Vector3{
        tangent1.x * local_direction.x + tangent2.x * local_direction.y + normal.x * local_direction.z,
        tangent1.y * local_direction.x + tangent2.y * local_direction.y + normal.y * local_direction.z,
        tangent1.z * local_direction.x + tangent2.z * local_direction.y + normal.z * local_direction.z
    };
}

Vector3 Shading_Context::world_to_local(const Vector3& world_direction) const {
    return Vector3 { 
        dot(world_direction, tangent1),
        dot(world_direction, tangent2),
        dot(world_direction, normal)
    };
}
