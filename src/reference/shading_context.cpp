#include "std.h"
#include "lib/common.h"
#include "shading_context.h"

#include "intersection.h"

#include "lib/render_object.h"

Shading_Context::Shading_Context(const Vector3& wo, const Intersection& intersection)
    : Wo(wo)
{
    if (intersection.geometry_type == Geometry_Type::triangle_mesh) {
        const Triangle_Intersection& ti = intersection.triangle_intersection;
        P = ti.mesh->get_position(ti.triangle_index, ti.b1, ti.b2);
        N = ti.mesh->get_normal(ti.triangle_index, ti.b1, ti.b2);
        UV = ti.mesh->get_uv(ti.triangle_index, ti.b1, ti.b2);

        if (intersection.render_object != nullptr) {
            P = transform_point(intersection.render_object->object_to_world_transform, P);
            N = transform_vector(intersection.render_object->object_to_world_transform, N);
            material = intersection.render_object->material;
            area_light = intersection.render_object->area_light;
        }
        else {
            material = Null_Material;
            area_light = Null_Light;
        }
    }
    else {
        ASSERT(false);
    }
}
