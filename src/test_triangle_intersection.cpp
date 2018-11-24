#include "common.h"
#include "intersection.h"
#include "ray.h"

void test_triangle_intersection()
{
    Ray ray(Vector3(0, 0, 0), Vector3(0, 1, 0));

    Vector3 p[3] = {
        Vector3(-0.5f, 0, -0.5f),
        Vector3( 0.5f, 0, -0.5f),
        Vector3( 0, 0, 0.5f)
    };

    const int N = 10'000'000;
    Triangle_Intersection isect;

    Timestamp t;

    float b1, b2;
    for (int i = 0; i < N; i++) {
        intersect_triangle_moller_trumbore(ray, p[0], p[1], p[2], b1, b2);
    }

    int64_t ns = elapsed_nanoseconds(t);

    double cpu_ghz = get_cpu_frequency_ghz();
    printf("CPU frequency = %.2f GHz\n", cpu_ghz);

    double milliseconds_for_all_triangles = ns / 1'000'000.0;
    printf("All triangles intersection time: %.3f milliseconds\n", milliseconds_for_all_triangles);

    double nanoseconds_per_triangle = ns / double(N);
    int clocks = static_cast<int>(nanoseconds_per_triangle * cpu_ghz);
    printf("Single triangle intersection time: %.2f nanoseconds, %d clocks\n", nanoseconds_per_triangle, clocks);
}
