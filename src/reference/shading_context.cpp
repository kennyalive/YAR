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
static bool adjust_shading_normal(const Vector3& Wo, const Vector3& Ng, Vector3* N) {
    // check renderer convention: shading frame is oriented in such way that Wo is in the positive hemisphere
    ASSERT(dot(Wo, Ng) >= 0.f);

    Vector3 R = reflect(Wo, *N);

    // If reflected direction is above the geometric surface then
    // shading normal adaptation is not needed.
    float a = dot(R, Ng);
    if (a >= 0.f)
        return false;

    // For almost tangential Wo we have catastrophic cancellation in 'Wo + tangent' expression below.
    // For this situation we know that the result will be close to geometric normal, so return it directly.
    if (dot(Wo, Ng) < 1e-3f) {
        *N = Ng;
        return true;
    }

    float b = dot(*N, Ng);
    ASSERT(b > 0.f);

    // This epsilon pulls tangent vector up a bit, so it's not strictly parallel to the surface.
    // In the scenario of ideal specular reflection it helps to avoid self intersection.
    float epsilon = 1e-4f;

    float distance_to_surface_along_normal = std::abs(a)/b;
    Vector3 tangent = R + (distance_to_surface_along_normal + epsilon) * (*N);
    tangent.normalize();

    // Check that tangent vector is either above the surface or only a little bit below it.
    // The second scenario is still possible in rare situations due to finite FP precision when we have
    // extreme orientation of the shading normal N relative to the geometric normal Ng (almost orthogonal).
    // This is not a problem per se, because tangent vector only provides characteristic orientation.
    ASSERT(dot(tangent, Ng) > -1e-5f);

    Vector3 new_N = (Wo + tangent).normalized();
    ASSERT(dot(Wo, new_N) >= 0.f);
    *N = new_N;
    return true;
}

void Shading_Context::initialize_from_intersection(
    const Scene_Context& global_ctx,
    Thread_Context& thread_ctx,
    const Ray& ray, const Auxilary_Rays* auxilary_rays, const Intersection& intersection)
{
    *this = Shading_Context{};

    Wo = -ray.direction;

    // Geometry_Type-specific initialization.
    //
    // The following fields should be initialized: P, N, Ng, UV, dPdu, dPdv, dNdu, dNdv.
    // The values should be calculated in the local coordinate system of the object.
    Vector3 dPdu, dPdv;
    if (intersection.geometry_type == Geometry_Type::triangle_mesh) {
        init_from_triangle_mesh_intersection(intersection.triangle_intersection);
    }
    else {
        ASSERT(false);
    }

    // Transform geometry data to world space.
    {
        const Matrix3x4& object_to_world = intersection.scene_object->object_to_world_transform;
        P    = transform_point (object_to_world, P);
        N    = transform_vector(object_to_world, N);
        Ng   = transform_vector(object_to_world, Ng);
        dPdu = transform_vector(object_to_world, dPdu);
        dPdv = transform_vector(object_to_world, dPdv);
        dNdu = transform_vector(object_to_world, dNdu);
        dNdv = transform_vector(object_to_world, dNdv);
    }

    // Trying to avoid self-shadowing.
    P = offset_ray_origin(P, Ng);

    if (auxilary_rays)
        calculate_UV_derivates(*auxilary_rays);

    // NOTE: adjustment of shading normal invalidates dNdu/dNdv. For now it's not clear to which degree
    // it could be an issue (in most cases shading normals are left unchanged). Until further evidence,
    // we assume that dNdu/dNdv is still a reasonable approximation.
    shading_normal_adjusted = adjust_shading_normal(Wo, Ng, &N);

    tangent2 = cross(N, dPdu).normalized();
    tangent1 = cross(tangent2, N);

    area_light = intersection.scene_object->area_light;

    // Init scattering information.
    if (intersection.scene_object->material != Null_Material) {
        Material_Handle mtl = intersection.scene_object->material;
        if (mtl.type == Material_Type::mirror) {
            mirror_surface = true;
            const Mirror_Material& params = global_ctx.materials.mirror[mtl.index];
            mirror_reflectance = evaluate_rgb_parameter(global_ctx, *this, params.reflectance);
        }
        else {
            bsdf = create_bsdf(global_ctx, thread_ctx, *this, intersection.scene_object->material);
        }
    }
}

void Shading_Context::init_from_triangle_mesh_intersection(const Triangle_Intersection& ti) {
    P = ti.mesh->get_position(ti.triangle_index, ti.b1, ti.b2);

    Vector3 p0, p1, p2;
    ti.mesh->get_triangle(ti.triangle_index, p0, p1, p2);
    Ng = cross(p1 - p0, p2 - p0).normalized();
    Ng = dot(Ng, Wo) < 0 ? -Ng : Ng;

    N = ti.mesh->get_normal(ti.triangle_index, ti.b1, ti.b2);
    N = dot(N, Ng) < 0 ? -N : N;

    UV = ti.mesh->get_uv(ti.triangle_index, ti.b1, ti.b2);

    Vector2 uvs[3];
    ti.mesh->get_uvs(ti.triangle_index, uvs);
    float a[2][2] = {
        { uvs[1].u - uvs[0].u, uvs[1].v - uvs[0].v },
        { uvs[2].u - uvs[0].u, uvs[2].v - uvs[0].v }
    };

    // dPdu/dPdv
    {
        Vector3 b[2] = {
            p1 - p0,
            p2 - p0
        };
        if (!solve_linear_system_2x2(a, b, &dPdu, &dPdv)) {
            coordinate_system_from_vector(Ng, &dPdu, &dPdv);
        }
    }
    // dNdu/dNdv
    {
        Vector3 n[3];
        ti.mesh->get_normals(ti.triangle_index, n);
        Vector3 b[2] = {
            n[1] - n[0],
            n[2] - n[0]
        };
        if (!solve_linear_system_2x2(a, b, &dNdu, &dNdv)) {
            coordinate_system_from_vector(Ng, &dNdu, &dNdv);
        }
    }
}

void Shading_Context::calculate_UV_derivates(const Auxilary_Rays& auxilary_rays) {
    // Compute position derivatives with respect to screen coordinates using auxilary offset rays.
    float plane_d = -dot(Ng, P);
    float tx = ray_plane_intersection(auxilary_rays.ray_dx_offset, Ng, plane_d);
    float ty = ray_plane_intersection(auxilary_rays.ray_dy_offset, Ng, plane_d);

    Vector3 px = auxilary_rays.ray_dx_offset.get_point(tx);
    Vector3 py = auxilary_rays.ray_dy_offset.get_point(ty);
    dPdx = px - P;
    dPdy = py - P;

    // Compute UV derivatives with respect to screen coordinates (PBRT, 10.1.1).
    //
    // We need to solve these two linear systems:
    //  dPdx = dPdu * dUdx + dPdv * dVdx (3 equations)
    //  dPdy = dPdu * dUdy + dPdv * dVdy (3 equations)

    // In a system of 3 linear equations with 2 unknown variables it's
    // possible that one equation is degenerate. Here we get rid of equation
    // with the highest chance to be degenerate.
    int dim0 = 0, dim1 = 1;
    {
        Vector3 a = Ng.abs();
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
        { dPdu[dim0], dPdv[dim0] },
        { dPdu[dim1], dPdv[dim1] }
    };
    float bx[2] = {
        dPdx[dim0],
        dPdx[dim1]
    };
    float by[2] = {
        dPdy[dim0],
        dPdy[dim1]
    };
    // If equation cannot be solved then derivative stay initialized to zero.
    solve_linear_system_2x2(a, bx, &dUVdx.u, &dUVdx.v);
    solve_linear_system_2x2(a, by, &dUVdy.u, &dUVdy.v);
}

float Shading_Context::compute_texture_lod(int mip_count, const Vector2& uv_scale) const {
    Vector2 dUVdx_scaled = dUVdx * uv_scale;
    Vector2 dUVdy_scaled = dUVdy * uv_scale;

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
        tangent1.x * local_direction.x + tangent2.x * local_direction.y + N.x * local_direction.z,
        tangent1.y * local_direction.x + tangent2.y * local_direction.y + N.y * local_direction.z,
        tangent1.z * local_direction.x + tangent2.z * local_direction.y + N.z * local_direction.z
    };
}

Vector3 Shading_Context::world_to_local(const Vector3& world_direction) const {
    return Vector3 { 
        dot(world_direction, tangent1),
        dot(world_direction, tangent2),
        dot(world_direction, N)
    };
}
