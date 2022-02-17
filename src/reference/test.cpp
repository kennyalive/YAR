#include "std.h"
#include "test.h"

void test_random();
void test_sampling();
void test_triangle_intersection();
void test_watertightness();
void test_kdtree();
void benchmark_triangle_intersection();
void benchmark_kdtree();

void run_tests(const std::string& test_name) {
    if (test_name.empty()) {
        test_random();
        test_sampling();
        test_triangle_intersection();
        test_watertightness();
        test_kdtree();
    }
    else if (test_name == "kdtree") {
        test_kdtree();
    }
    else if (test_name == "bench_intersection") {
        benchmark_triangle_intersection();
    }
    else if (test_name == "bench_kdtree") {
        benchmark_kdtree();
    }
    else {
        printf("run_tests: Unknown test name: %s\n", test_name.c_str());
    }
}
