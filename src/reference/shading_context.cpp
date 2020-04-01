#include "std.h"
#include "lib/common.h"
#include "shading_context.h"

#include "intersection.h"
#include "scattering.h"

#include "lib/math.h"
#include "lib/scene_object.h"

Shading_Context::Shading_Context(
    const Render_Context& global_ctx,
    const Shading_Point_Rays& rays, const Intersection& intersection,
    void* bsdf_allocation, int bsdf_allocation_size)
{
    Wo = -rays.incident_ray.direction;

    if (intersection.geometry_type == Geometry_Type::triangle_mesh) {
        // TODO: move this code to separate function so we will have shape dependent computation
        // in dedicated functions and then we will do shape independent computations common for all shapes.
        const Triangle_Intersection& ti = intersection.triangle_intersection;
        P = ti.mesh->get_position(ti.triangle_index, ti.b1, ti.b2);
        N = ti.mesh->get_normal(ti.triangle_index, ti.b1, ti.b2);
        UV = ti.mesh->get_uv(ti.triangle_index, ti.b1, ti.b2);

        Vector3 p0, p1, p2;
        ti.mesh->get_triangle(ti.triangle_index, p0, p1, p2);
        Ng = cross(p1 - p0, p2 - p0);

        if (intersection.scene_object != nullptr) {
            P = transform_point(intersection.scene_object->object_to_world_transform, P);
            Ng = transform_vector(intersection.scene_object->object_to_world_transform, Ng).normalized();
            N = transform_vector(intersection.scene_object->object_to_world_transform, N);
            area_light = intersection.scene_object->area_light;
        }
        else {
            Ng.normalize();
        }
        Ng = dot(Ng, N) < 0 ? -Ng : Ng;

        // Compute dPdu/dPdv
        {
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
            if (!solve_linear_system_2x2(a, b, &dPdu, &dPdv)) {
                coordinate_system_from_vector(Ng, &dPdu, &dPdv);
            }
        }
    }
    else {
        ASSERT(false);
    }

    // Compute position derivatives with respect to screen coordinates using auxilary offset rays.
    Vector3 dPdx, dPdy;
    {
        float plane_d = -dot(Ng, P);
        float tx = ray_plane_intersection(rays.auxilary_ray_dx_offset, Ng, plane_d);
        float ty = ray_plane_intersection(rays.auxilary_ray_dy_offset, Ng, plane_d);

        Vector3 px = rays.auxilary_ray_dx_offset.get_point(tx);
        Vector3 py = rays.auxilary_ray_dy_offset.get_point(ty);
        dPdx = px - P;
        dPdy = py - P;
    }

    // Compute UV derivatives with respect to screen coordinates (PBRT, 10.1.1).
    {
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
        if (!solve_linear_system_2x2(a, bx, &dUVdx.u, &dUVdx.v)) {
            dUVdx = Vector2();
        }
        if (!solve_linear_system_2x2(a, by, &dUVdy.u, &dUVdy.v)) {
            dUVdy = Vector2();
        }
    }

    // Allocate BSDF.
    if (intersection.scene_object != nullptr && intersection.scene_object->material != Null_Material) {
        bsdf = create_bsdf(global_ctx, *this, intersection.scene_object->material, bsdf_allocation, bsdf_allocation_size);
    }
}
