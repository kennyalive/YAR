#include "std.h"
#include "lib/common.h"
#include "intersection.h"
#include "intersection_simd.h"
#include "lib/random.h"

void test_triangle_intersection()
{
    printf("-------------\n");
    printf("Test: test_triangle_intersection\n");

    const int N = 100'000'000;
    printf("Checking %d ray-triangle intersections\n", N);
    printf("Intersection point on triangle is compared to corresponding point on the ray (with 1mm precision)\n");

    int fail_count_watertight = 0;
    int fail_count_watertight_8x = 0;
    int fail_count_watertight_4x = 0;
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
            Vector3 b;
            float t = intersect_triangle_watertight(ray, p0, p1, p2, &b);
            if (t != Infinity) {
                Vector3 isect_p = b[0]*p0 + b[1]*p1 + b[2]*p2;
                Vector3 isect_p2 = ray.origin + t * ray.direction;
                float delta = (isect_p - isect_p2).length();
                fail_count_watertight += (delta > 1e-3f); // 1mm precision for ~1m sized triangles
            }
        }

        // 8x SIMD implementation of Watertight algorithm.
        {
            __m256 px[3];
            __m256 py[3];
            __m256 pz[3];

            px[0] = _mm256_set1_ps(p0.x);
            px[1] = _mm256_set1_ps(p1.x);
            px[2] = _mm256_set1_ps(p2.x);

            py[0] = _mm256_set1_ps(p0.y);
            py[1] = _mm256_set1_ps(p1.y);
            py[2] = _mm256_set1_ps(p2.y);

            pz[0] = _mm256_set1_ps(p0.z);
            pz[1] = _mm256_set1_ps(p1.z);
            pz[2] = _mm256_set1_ps(p2.z);

            float t;
            Vector3 b;
            uint32_t index;
            Triangle_Intersection_8x isect8 = intersect_triangle_watertight_8x(ray, px, py, pz);
            isect8.reduce(&t, &b, &index);
            if (t != Infinity) {
                Vector3 isect_p = b[0] * p0 + b[1] * p1 + b[2] * p2;
                Vector3 isect_p2 = ray.origin + t * ray.direction;
                float delta = (isect_p - isect_p2).length();
                fail_count_watertight_8x += (delta > 1e-3f); // 1mm precision for ~1m sized triangles
            }
        }

        // 4x SIMD implementation of Watertight algorithm.
        {
            __m128 px[3];
            __m128 py[3];
            __m128 pz[3];

            px[0] = _mm_set1_ps(p0.x);
            px[1] = _mm_set1_ps(p1.x);
            px[2] = _mm_set1_ps(p2.x);

            py[0] = _mm_set1_ps(p0.y);
            py[1] = _mm_set1_ps(p1.y);
            py[2] = _mm_set1_ps(p2.y);

            pz[0] = _mm_set1_ps(p0.z);
            pz[1] = _mm_set1_ps(p1.z);
            pz[2] = _mm_set1_ps(p2.z);

            float t;
            Vector3 b;
            uint32_t index;
            Triangle_Intersection_4x isect4 = intersect_triangle_watertight_4x(ray, px, py, pz);
            isect4.reduce(&t, &b, &index);
            if (t != Infinity) {
                Vector3 isect_p = b[0] * p0 + b[1] * p1 + b[2] * p2;
                Vector3 isect_p2 = ray.origin + t * ray.direction;
                float delta = (isect_p - isect_p2).length();
                fail_count_watertight_4x += (delta > 1e-3f); // 1mm precision for ~1m sized triangles
            }
        }

        // Möller-trumbore algorithm.
        {
            Vector3 b;
            float t = intersect_triangle_möller_trumbore(ray, p0, p1, p2, &b);
            if (t != Infinity) {
                Vector3 isect_p = b[0]*p0 + b[1]*p1 + b[2]*p2;
                Vector3 isect_p2 = ray.origin + t * ray.direction;
                float delta = (isect_p - isect_p2).length();
                fail_count_möller_trumbore += (delta > 1e-3f); // 1mm precision for ~1m sized triangles
            }
        }
    }
    printf("Fail count (watertight): %d\n", fail_count_watertight);
    printf("Fail count (watertight_8x): %d\n", fail_count_watertight_8x);
    printf("Fail count (watertight_4x): %d\n", fail_count_watertight_4x);
    printf("Fail count (möller-trumbore): %d\n", fail_count_möller_trumbore);
}

void test_simd_triangle_intersection()
{
#define CHECK(expr) if (expr) {} else error("Failure! Failed expression: %s", #expr)

    printf("-------------\n");
    printf("Test: test_simd_triangle_intersection\n");

    {
        __m256 px[3];
        __m256 py[3];
        __m256 pz[3];

        px[0] = _mm256_set_ps(-1, -1, -8, -1, -1, -1, -1, 10);
        px[1] = _mm256_set_ps( 1,  1, -6,  1,  1,  1,  1, 12);
        px[2] = _mm256_set_ps( 0,  0, -7,  0,  0,  0,  0, 11);

        py[0] = _mm256_set1_ps(-1.f);
        py[1] = _mm256_set1_ps(-1.f);
        py[2] = _mm256_set1_ps(2.f);

        pz[0] = _mm256_set1_ps(0.f);
        pz[1] = _mm256_set1_ps(0.f);
        pz[2] = _mm256_set1_ps(0.f);

        Ray ray;
        ray.origin = Vector3(0, 0, 5);
        ray.direction = Vector3(0, 0, -1);

        Triangle_Intersection_8x isect8 = intersect_triangle_watertight_8x(ray, px, py, pz);
        alignas(32) float t[8];
        _mm256_store_ps(t, isect8.t);
        for (int i = 0; i < 8; i++) {
            if (i != 0 && i != 5)
                CHECK(std::abs(t[i] - 5.f) < 1e-5f);
            else
                CHECK(t[i] == Infinity);
        }
    }

    {
        __m128 px[3];
        __m128 py[3];
        __m128 pz[3];

        px[0] = _mm_set_ps(-1, -1, -8, -1);
        px[1] = _mm_set_ps( 1,  1, -6,  1);
        px[2] = _mm_set_ps( 0,  0, -7,  0);

        py[0] = _mm_set1_ps(-1.f);
        py[1] = _mm_set1_ps(-1.f);
        py[2] = _mm_set1_ps(2.f);

        pz[0] = _mm_set1_ps(0.f);
        pz[1] = _mm_set1_ps(0.f);
        pz[2] = _mm_set1_ps(0.f);

        Ray ray;
        ray.origin = Vector3(0, 0, 5);
        ray.direction = Vector3(0, 0, -1);

        Triangle_Intersection_4x isect4 = intersect_triangle_watertight_4x(ray, px, py, pz);
        alignas(16) float t[4];
        _mm_store_ps(t, isect4.t);
        for (int i = 0; i < 4; i++) {
            if (i != 1)
                CHECK(std::abs(t[i] - 5.f) < 1e-5f);
            else
                CHECK(t[i] == Infinity);
        }
    }

    {
        __m256 px[3];
        __m256 py[3];
        __m256 pz[3];

        px[0] = _mm256_set_ps(-1, -1, -8, -1, -1, -1, -1, 10);
        px[1] = _mm256_set_ps( 1,  1, -6,  1,  1,  1,  1, 12);
        px[2] = _mm256_set_ps( 0,  0, -7,  0,  0,  0,  0, 11);

        py[0] = _mm256_set1_ps(-1.f);
        py[1] = _mm256_set1_ps(-1.f);
        py[2] = _mm256_set1_ps(2.f);

        pz[0] = _mm256_set_ps(-5, -6, 3, -1, 2, 1, -1, 0);
        pz[1] = _mm256_set_ps(-5, -6, 3, -1, 2, 1, -1, 0);
        pz[2] = _mm256_set_ps(-5, -6, 3, -1, 2, 1, -1, 0);

        Ray ray;
        ray.origin = Vector3(0, 0, 5);
        ray.direction = Vector3(0, 0, -1);

        Triangle_Intersection_8x isect8 = intersect_triangle_watertight_8x(ray, px, py, pz);
        alignas(32) float t[8];
        _mm256_store_ps(t, isect8.t);
        for (int i = 0; i < 8; i++) {
            if (i != 0 && i != 5)
                CHECK(std::abs(5 - t[i] - pz[0].m256_f32[i]) < 1e-5f);
            else
                CHECK(t[i] == Infinity);

        }

        float closest_distance;
        Vector3 b;
        uint32_t index;
        isect8.reduce(&closest_distance, &b, &index);
        CHECK(std::abs(closest_distance - 3.f) < 1e-5f);
    }

    {
        __m128 px[3];
        __m128 py[3];
        __m128 pz[3];

        px[0] = _mm_set_ps(-1, -1, -8, -1);
        px[1] = _mm_set_ps( 1,  1, -6,  1);
        px[2] = _mm_set_ps( 0,  0, -7,  0);

        py[0] = _mm_set1_ps(-1.f);
        py[1] = _mm_set1_ps(-1.f);
        py[2] = _mm_set1_ps(2.f);

        pz[0] = _mm_set_ps(-5, -6, 3, -1);
        pz[1] = _mm_set_ps(-5, -6, 3, -1);
        pz[2] = _mm_set_ps(-5, -6, 3, -1);

        Ray ray;
        ray.origin = Vector3(0, 0, 5);
        ray.direction = Vector3(0, 0, -1);

        Triangle_Intersection_4x isect4 = intersect_triangle_watertight_4x(ray, px, py, pz);
        alignas(16) float t[4];
        _mm_store_ps(t, isect4.t);
        for (int i = 0; i < 4; i++) {
            if (i != 1)
                CHECK(std::abs(5 - t[i] - pz[0].m128_f32[i]) < 1e-5f);
            else
                CHECK(t[i] == Infinity);

        }

        float closest_distance;
        Vector3 b;
        uint32_t index;
        isect4.reduce(&closest_distance, &b, &index);
        CHECK(std::abs(closest_distance - 6.f) < 1e-5f);
    }

    printf("Success\n");
#undef CHECK
}
