#include "std.h"
#include "lib/common.h"
#include "intersection.h"
#include "lib/ray.h"

void benchmark_triangle_intersection()
{
    Ray ray{Vector3(0, 0, 0), Vector3(0, 1, 0)};
    Vector3 p0 = Vector3(-5, 20, -5);
    Vector3 p1 = Vector3( 5, 20, -5);
    Vector3 p2 = Vector3( 0, 20, 5);

    const int N = 50'000'000;
    const double cpu_freq_ghz = get_cpu_frequency_ghz();

    printf("-------------\n");
    printf("Benchmark: triangle_intersection\n");
    printf("Benchmark measures time to shoot %.1fM rays against a triangle\n", N / 1e6f);
    printf("Benchmark assumes CPU frequency is %.2f GHz\n", cpu_freq_ghz);
    printf("Möller-Trumbore algorithm:\n");
    {
        Timestamp t;
        Vector3 b;
        for (int i = 0; i < N/4; i++) {
            intersect_triangle_möller_trumbore(ray, p0, p1, p2, &b);
            intersect_triangle_möller_trumbore(ray, p0, p1, p2, &b);
            intersect_triangle_möller_trumbore(ray, p0, p1, p2, &b);
            intersect_triangle_möller_trumbore(ray, p0, p1, p2, &b);
        }
        int64_t ns = elapsed_nanoseconds(t);

        double milliseconds_for_all_triangles = ns / 1'000'000.0;
        printf("  Total time: %.3f milliseconds\n", milliseconds_for_all_triangles);
        double nanoseconds_per_triangle = ns / double(N);
        float clocks = static_cast<float>(nanoseconds_per_triangle * cpu_freq_ghz);
        printf("  Single triangle time: %.2f nanoseconds, %.1f clocks\n", nanoseconds_per_triangle, clocks);
    }

    printf("Watertight algorithm:\n");
    {
        Timestamp t;
        Vector3 b;
        for (int i = 0; i < N/4; i++) {
            intersect_triangle_watertight(ray, p0, p1, p2, &b);
            intersect_triangle_watertight(ray, p0, p1, p2, &b);
            intersect_triangle_watertight(ray, p0, p1, p2, &b);
            intersect_triangle_watertight(ray, p0, p1, p2, &b);
        }
        int64_t ns = elapsed_nanoseconds(t);

        double milliseconds_for_all_triangles = ns / 1'000'000.0;
        printf("  Total time: %.3f milliseconds\n", milliseconds_for_all_triangles);
        double nanoseconds_per_triangle = ns / double(N);
        float clocks = static_cast<float>(nanoseconds_per_triangle * cpu_freq_ghz);
        printf("  Single triangle time: %.2f nanoseconds, %.1f clocks\n", nanoseconds_per_triangle, clocks);
    }
}
