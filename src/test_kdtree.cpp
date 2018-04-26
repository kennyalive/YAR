#include "common.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "rng.h"
#include "test_ray_generator.h"
#include "triangle_mesh.h"
#include "triangle_mesh_loader.h"
#include "vector.h"

#ifdef _WIN32
#include <pmmintrin.h>
#endif

#include <string>
#include <vector>

enum { benchmark_ray_count = 1'000'000 };
enum { debug_rays = false };
enum { debug_ray_count = 4 };

//const std::string model_path = "data/soccer_ball.stl";
//const std::string kdtree_path = "data/soccer_ball.kdtree";
//const int validation_ray_count = 32768;

//const std::string model_path = "data/teapot.stl";
//const std::string kdtree_path = "data/teapot.kdtree";
//const int validation_ray_count = 1'000'000;

const std::string model_path = "data/bunny.stl";
const std::string kdtree_path = "data/bunny.kdtree";
const int validation_ray_count = 10'000;

//const std::string model_path = "data/dragon.stl";
//const std::string kdtree_path = "data/dragon.kdtree";
//const int validation_ray_count = 5'000;

const double cpu_ghz = 4.5; // get_base_cpu_frequency_ghz();

int benchmark_kd_tree(const KdTree& kdtree) {
    Timestamp tx;

    auto bounds = kdtree.get_mesh().get_bounds();

    Vector last_hit = (bounds.min_p + bounds.max_p) * 0.5f;
    float last_hit_epsilon = 0.0;
    auto ray_generator = Ray_Generator(bounds);

    int64_t time_ns = 0;

    for (int i = 0; i < benchmark_ray_count; i++) {
        const Ray ray = ray_generator.generate_ray(last_hit, last_hit_epsilon);

        Timestamp t2;

        KdTree::Intersection intersection;
        bool hitFound = kdtree.intersect(ray, intersection);

        time_ns += elapsed_nanoseconds(t2);

        if (hitFound) {
            last_hit = ray.get_point(intersection.t);
            last_hit_epsilon = intersection.epsilon;
        }

        if (debug_rays && i < debug_ray_count) {
            if (hitFound)
                printf("%d: found: %s, lastHit: %.14f %.14f %.14f\n", i,
                    hitFound ? "true" : "false", last_hit.x, last_hit.y, last_hit.z);
            else
                printf("%d: found: %s\n", i, hitFound ? "true" : "false");
        }
    }

    double nanoseconds_per_raycast = time_ns / double(benchmark_ray_count);
    int clocks = static_cast<int>(nanoseconds_per_raycast * cpu_ghz);
    printf("Single raycast time: %.2f nanoseconds, %d clocks\n", nanoseconds_per_raycast, clocks);

    //return static_cast<int>(elapsed_milliseconds(t));
    return (int)(time_ns / 1'000'000);
}

void validate_kdtree(const KdTree& kdtree, int ray_count) {
    printf("Running kdtree validation... ");
    auto bounds = kdtree.get_mesh().get_bounds();
    Vector last_hit = (bounds.min_p + bounds.max_p) * 0.5;
    float last_hit_epsilon = 0.0f;

    auto ray_generator = Ray_Generator(bounds);

    for (int i = 0; i < ray_count; i++) {
        const Ray ray = ray_generator.generate_ray(last_hit, last_hit_epsilon);

        KdTree::Intersection kdtree_intersection;
        bool kdtree_hit = kdtree.intersect(ray, kdtree_intersection);

        KdTree::Intersection brute_force_intersection;
        bool brute_force_hit = false;

        int hit_k = -1;

        for (int32_t k = 0; k < kdtree.get_mesh().get_triangle_count(); k++) {
            Triangle triangle = kdtree.get_mesh().get_triangle(k);

            Triangle_Intersection intersection;
            bool hit = intersect_triangle(ray, triangle, intersection);

            if (hit && intersection.t < brute_force_intersection.t) {
                brute_force_intersection.t = intersection.t;
                brute_force_hit = true;
                hit_k = k;
            }
        }

        if (kdtree_hit != brute_force_hit || kdtree_intersection.t != brute_force_intersection.t) {
            printf("KdTree accelerator test failure:\n"
                "Rays validated so far: %d (%.2f%%)\n"
                "KdTree hit: %s\n"
                "actual hit: %s\n"
                "KdTree T %.16g [%a]\n"
                "actual T %.16g [%a]\n"
                "ray origin: (%a, %a, %a)\n"
                "ray direction: (%a, %a, %a)\n",
                i, float(i) / float(ray_count),
                kdtree_hit ? "true" : "false",
                brute_force_hit ? "true" : "false", kdtree_intersection.t,
                kdtree_intersection.t, brute_force_intersection.t,
                brute_force_intersection.t, ray.o.x, ray.o.y, ray.o.z, ray.d.x, ray.d.y, ray.d.z
            );
            error("KdTree traversal error detected");
        }

        if (kdtree_hit) {
            last_hit = ray.get_point(kdtree_intersection.t);
            last_hit_epsilon = kdtree_intersection.epsilon;
        }
    }

    printf("DONE\n");
}

template <typename T> struct Triangle_Mesh_Selector;

template <>
struct Triangle_Mesh_Selector<Indexed_Triangle_Mesh> {
    Triangle_Mesh_Selector(Indexed_Triangle_Mesh& indexed_mesh) : mesh(indexed_mesh) {}
    Indexed_Triangle_Mesh& mesh;
};

template <>
struct Triangle_Mesh_Selector<Simple_Triangle_Mesh> {
    Triangle_Mesh_Selector(Indexed_Triangle_Mesh& indexed_mesh) : mesh(Simple_Triangle_Mesh::from_indexed_mesh(indexed_mesh)) {}
    Simple_Triangle_Mesh mesh;
};

const bool build_tree = false;

void test_kdtree() {
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
    
    RNG rng;

    std::unique_ptr<Indexed_Triangle_Mesh> indexed_mesh = LoadTriangleMesh(model_path);

    Triangle_Mesh_Selector<Triangle_Mesh> mesh_selector(*indexed_mesh);
    Triangle_Mesh& mesh = mesh_selector.mesh;

    if (build_tree) {
        KdTree_Build_Params params;
        Timestamp t;
        KdTree kdtree = build_kdtree(mesh, params);
        int time = int(elapsed_milliseconds(t));
        printf("KdTree build time = %dms\n", time);
		kdtree.calculate_stats().print();

        kdtree.save_to_file("test.kdtree");
        printf("\n");
        return;
    }
    //auto kdtree = std::unique_ptr<KdTree>(new KdTree(kdtree_path, mesh));
    auto kdtree = std::unique_ptr<KdTree>(new KdTree("test.kdtree", mesh));

    mesh.print_info();
    kdtree->calculate_stats().print();
    printf("\n");
    printf("=========================\n");
    printf("shooting rays (kdtree)...\n");

    int timeMsec = benchmark_kd_tree(*kdtree);
    double speed = (benchmark_ray_count / 1000000.0) / (timeMsec / 1000.0);
    printf("raycast performance [%-6s]: %.2f MRays/sec, (rnd = %d)\n", model_path.c_str(), speed, rng.random_uint32());

    validate_kdtree(*kdtree, validation_ray_count);
}
