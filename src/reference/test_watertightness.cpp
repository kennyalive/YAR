#include "std.h"
#include "lib/common.h"
#include "lib/tessellation.h"
#include "kdtree_builder.h"

void test_watertightness() {
    printf("-------------\n");
    printf("Test: test_watertightness\n");
    printf("Testing intersect_triangle_watertight() for watertightness...\n");

    float radius = 0.5f;
    Geometries geometries;
    {
        Triangle_Mesh mesh = create_sphere_mesh(radius, 6, false);
        geometries.triangle_meshes.push_back(std::move(mesh));
    }
    Geometry_KdTree kdtree = build_geometry_kdtree(&geometries, {Geometry_Type::triangle_mesh, 0});

    const int ray_count = 100'000'000;
    {
        printf("Shooting %d rays against sphere of radius %.2f cm\n", ray_count, radius * 100);
        float average_triangle_area = 4 * Pi * radius * radius / geometries.triangle_meshes[0].get_triangle_count();
        float triangle_characteristic_size = std::sqrt(average_triangle_area * 4.f / std::sqrt(3.f));
        printf("Sphere's triangle characteristic size %.3f mm\n", triangle_characteristic_size * 1000);
    }

    const Vector3 ray_origin = Vector3(0, 0, 2);
    ASSERT(ray_origin.z > radius);
    const float point_generation_radius = 0.75f * radius;

    int watertightness_violation_count = 0;

    RNG rng;
    rng.init(0, 0);

    for (uint32_t i = 0; i < ray_count; i++) {
        // uniformly generate points insice a circle
        float phi = 2 * Pi * rng.get_float();
        float r = point_generation_radius * std::sqrt(rng.get_float());
        float x = r * std::cos(phi);
        float y = r * std::sin(phi);

        Vector3 ray_direction = (Vector3(x, y, 0) - ray_origin).normalized();
        Ray ray{ ray_origin, ray_direction };

        Intersection isect;
        bool hit_found = kdtree.intersect(ray, isect);
        ASSERT(hit_found);
        Vector3 isect_point = ray.get_point(isect.t);
        ASSERT(isect_point.z < radius + 1e-4f);

        if (isect_point.z <= 0.f)
        {
            printf("Found watertightness violation, intersection point: z == %f (should be positive), xy == (%f, %f)\n",
                isect_point.z, isect_point.x, isect_point.y
            );
            watertightness_violation_count++;
        }
    }

    printf("Watertighness violation count: %d\n", watertightness_violation_count);
}
