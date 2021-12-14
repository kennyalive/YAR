#include "std.h"
#include "test.h"

void test_random();
void test_sampling();
void test_triangle_intersection();
void test_watertightness();
void test_kdtree();
void benchmark_triangle_intersection();
void benchmark_kdtree();

void run_tests() {
    test_random();
    test_sampling();
    test_triangle_intersection();
    test_watertightness();
    test_kdtree();
    benchmark_triangle_intersection();
    benchmark_kdtree();
}
