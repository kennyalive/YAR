#include "std.h"
#include "lib/common.h"
#include "shading_context.h"

#include "intersection.h"
#include "scattering.h"

#include "lib/scene_object.h"

Shading_Context::Shading_Context(const Vector3& wo, const Intersection& intersection, const Materials& materials, void* bsdf_allocation, int bsdf_allocation_size)
    : Wo(wo)
{
    if (intersection.geometry_type == Geometry_Type::triangle_mesh) {
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
    }
    else {
        ASSERT(false);
    }

    if (intersection.scene_object != nullptr && intersection.scene_object->material != Null_Material) {
        bsdf = create_bsdf(*this, materials, intersection.scene_object->material, bsdf_allocation, bsdf_allocation_size);
    }
}
