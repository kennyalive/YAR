#include "std.h"
#include "test.h"

void test_random();
void test_sampling();
void test_triangle_intersection();
void test_simd_triangle_intersection();
void test_watertightness();
void test_kdtree();
void test_string();
void benchmark_triangle_intersection();
void benchmark_kdtree();
void benchmark_pbrt_parser();
void benchmark_string();

void run_tests(const char* test_name) {
    if (test_name[0] == 0) {
        test_random();
        test_sampling();
        test_triangle_intersection();
        test_simd_triangle_intersection();
        test_watertightness();
        test_kdtree();
        test_string();
    }
    else if (strcmp(test_name, "string") == 0) {
        test_string();
    }
    else if (strcmp(test_name, "intersection") == 0) {
        test_triangle_intersection();
    }
    else if (strcmp(test_name, "simd") == 0) {
        test_simd_triangle_intersection();
    }
    else if (strcmp(test_name, "kdtree") == 0) {
        test_kdtree();
    }
    else if (strcmp(test_name, "bench_intersection") == 0) {
        benchmark_triangle_intersection();
    }
    else if (strcmp(test_name, "bench_kdtree") == 0) {
        benchmark_kdtree();
    }
    else if (strcmp(test_name, "bench_pbrt_parser") == 0) {
        benchmark_pbrt_parser();
    }
    else if (strcmp(test_name, "bench_string") == 0) {
        benchmark_string();
    }
    else {
        printf("run_tests: Unknown test name: %s\n", test_name);
    }
}
