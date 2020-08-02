#include "std.h"
#include "lib/common.h"
#include "shading_context.h"

#include "bsdf.h"
#include "context.h"
#include "intersection.h"
#include "parameter_evaluation.h"

#include "lib/math.h"
#include "lib/scene_object.h"

Shading_Context::Shading_Context(
    const Scene_Context& global_ctx,
    Thread_Context& thread_ctx,
    const Shading_Point_Rays& rays, const Intersection& intersection)
{
    Wo = -rays.incident_ray.direction;

    // Geometry_Type-specific initialization.
    //
    // The fields to set: P, N, Ng, UV.
    // The output parameters to set: dPdu, dPdv
    // The values should be calculated in the local coordinate system of the object.
    Vector3 dPdu, dPdv;
    if (intersection.geometry_type == Geometry_Type::triangle_mesh) {
        init_from_triangle_mesh_intersection(intersection.triangle_intersection, &dPdu, &dPdv);
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
    }

    calculate_UV_derivates(rays, dPdu, dPdv);

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

void Shading_Context::init_from_triangle_mesh_intersection(const Triangle_Intersection& ti, Vector3* dPdu, Vector3* dPdv) {
    P = ti.mesh->get_position(ti.triangle_index, ti.b1, ti.b2);

    Vector3 p0, p1, p2;
    ti.mesh->get_triangle(ti.triangle_index, p0, p1, p2);
    Ng = cross(p1 - p0, p2 - p0).normalized();
    Ng = dot(Ng, Wo) < 0 ? -Ng : Ng;

    N = ti.mesh->get_normal(ti.triangle_index, ti.b1, ti.b2);
    N = dot(N, Ng) < 0 ? -N : N;

    UV = ti.mesh->get_uv(ti.triangle_index, ti.b1, ti.b2);

    // dPdu/dPdv
    Vector2 uvs[3];
    ti.mesh->get_uvs(ti.triangle_index, uvs);
    float a[2][2] = {
        { uvs[1].u - uvs[0].u, uvs[1].v - uvs[0].v },
        { uvs[2].u - uvs[0].u, uvs[2].v - uvs[0].v }
    };
    Vector3 b[2] = {
        p1 - p0,
        p2 - p0
    };
    if (!solve_linear_system_2x2(a, b, dPdu, dPdv)) {
        coordinate_system_from_vector(Ng, dPdu, dPdv);
    }
}

void Shading_Context::calculate_UV_derivates(const Shading_Point_Rays& rays, const Vector3& dPdu, const Vector3& dPdv) {
    // Compute position derivatives with respect to screen coordinates using auxilary offset rays.
    float plane_d = -dot(Ng, P);
    float tx = ray_plane_intersection(rays.auxilary_ray_dx_offset, Ng, plane_d);
    float ty = ray_plane_intersection(rays.auxilary_ray_dy_offset, Ng, plane_d);

    Vector3 px = rays.auxilary_ray_dx_offset.get_point(tx);
    Vector3 py = rays.auxilary_ray_dy_offset.get_point(ty);
    Vector3 dPdx = px - P;
    Vector3 dPdy = py - P;

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
