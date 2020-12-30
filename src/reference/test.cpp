#include "std.h"
#include "test.h"

void test_random();
void test_sampling();
void test_kdtree();

void run_tests() {
    test_random();
    test_sampling();
    test_kdtree();
}
