#include "std.h"
#include "lib/common.h"
#include "intersection.h"
#include "lib/random.h"

void test_triangle_intersection()
{
    printf("-------------\n");
    printf("Test: test_triangle_intersection\n");

    const int N = 100'000'000;
    printf("Checking %d ray-triangle intersections\n", N);
    printf("Intersection point on triangle is compared to corresponding point on the ray (with 1mm precision)\n");

    int fail_count_watertight = 0;
    int fail_count_möller_trumbore = 0;
    RNG rng;

    for (int i = 0; i < N; i++) {
        Vector3 p0 = Vector3{ rng.get_float(), rng.get_float(), rng.get_float() };
        Vector3 p1 = Vector3{ rng.get_float(), rng.get_float(), rng.get_float() };
        Vector3 p2 = Vector3{ rng.get_float(), rng.get_float(), rng.get_float() };

        Ray ray;
        {
            const float scale_factor = 1.05f;
            float b0 = rng.get_float();
            float b1 = (1.f - b0) * rng.get_float();
            float b2 = 1.f - b0 - b1;
            ASSERT(b2 >= 0);
            Vector3 point_on_ray = scale_factor * (b0*p0 + b1*p1 + b2*p2);

            ray.origin = Vector3{ rng.get_float() * 10.f - 20.f, rng.get_float() * 10.f - 20.f, rng.get_float() * 10.f - 20.f };
            ray.direction = (point_on_ray - ray.origin).normalized();
        }

        // Watertight algorithm.
        {
            float isect_b1, isect_b2;
            float t = intersect_triangle_watertight(ray, p0, p1, p2, Infinity, isect_b1, isect_b2);
            if (t != Infinity) {
                Vector3 isect_p = (1.f - isect_b1 - isect_b2)*p0 + isect_b1*p1 + isect_b2*p2;
                Vector3 isect_p2 = ray.origin + t * ray.direction;
                float delta = (isect_p - isect_p2).length();
                fail_count_watertight += (delta > 1e-3f); // 1mm precision for ~1m sized triangles
            }
        }
        // Möller-trumbore algorithm.
        {
            float isect_b1, isect_b2;
            float t = intersect_triangle_möller_trumbore(ray, p0, p1, p2, isect_b1, isect_b2);
            if (t != Infinity) {
                Vector3 isect_p = (1.f - isect_b1 - isect_b2)*p0 + isect_b1*p1 + isect_b2*p2;
                Vector3 isect_p2 = ray.origin + t * ray.direction;
                float delta = (isect_p - isect_p2).length();
                fail_count_möller_trumbore += (delta > 1e-3f); // 1mm precision for ~1m sized triangles
            }
        }
    }
    printf("Fail count (watertight): %d\n", fail_count_watertight);
    printf("Fail count (möller-trumbore): %d\n", fail_count_möller_trumbore);
}
