#include "lib/common.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "test_ray_generator.h"
#include "triangle_mesh.h"
#include "triangle_mesh_loader.h"
#include "lib/random.h"
#include "lib/vector.h"

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

const std::string model_path = "data/teapot.stl";
const std::string kdtree_path = "data/teapot.kdtree";
const int validation_ray_count = 1'000'000;

//const std::string model_path = "data/bunny.stl";
//const std::string kdtree_path = "data/bunny.kdtree";
//const int validation_ray_count = 10'000;

//const std::string model_path = "data/dragon.stl";
//const std::string kdtree_path = "data/dragon.kdtree";
//const int validation_ray_count = 5'000;

const double cpu_ghz = 4.5; // get_base_cpu_frequency_ghz();

int benchmark_kd_tree(const Mesh_KdTree& kdtree) {
    Timestamp tx;

    auto bounds = kdtree.get_bounds();

    Vector3 last_hit = (bounds.min_p + bounds.max_p) * 0.5f;
    float last_hit_epsilon = 0.0;
    auto ray_generator = Ray_Generator(bounds);

    int64_t time_ns = 0;

    for (int i = 0; i < benchmark_ray_count; i++) {
        const Ray ray = ray_generator.generate_ray(last_hit, last_hit_epsilon);

        Timestamp t2;

        Local_Geometry local_geom;
        float hit_distance = kdtree.intersect(ray, local_geom);
        bool hitFound = hit_distance != Infinity;

        time_ns += elapsed_nanoseconds(t2);

        if (hitFound) {
            last_hit = local_geom.position;
            last_hit_epsilon = hit_distance * 1e-3f;
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

void validate_kdtree(const Mesh_KdTree& kdtree, int ray_count) {
    printf("Running kdtree validation... ");
    auto bounds = kdtree.get_bounds();
    Vector3 last_hit = (bounds.min_p + bounds.max_p) * 0.5;
    float last_hit_epsilon = 0.0f;

    auto ray_generator = Ray_Generator(bounds);

    for (int i = 0; i < ray_count; i++) {
        const Ray ray = ray_generator.generate_ray(last_hit, last_hit_epsilon);

        Local_Geometry kdtree_intersection;
        float kdtree_hit_distance = kdtree.intersect(ray, kdtree_intersection);

        Triangle_Intersection brute_force_intersection;

        int hit_k = -1;
        for (int32_t k = 0; k < kdtree.get_primitive_source().get_primitive_count(); k++) {
            float old_t = brute_force_intersection.t;

            intersect_triangle(ray, kdtree.get_primitive_source().mesh, k, brute_force_intersection);

            if (brute_force_intersection.t != old_t) {
                hit_k = k;
            }
        }

        if (kdtree_hit_distance != brute_force_intersection.t) {
            const auto& o = ray.origin;
            const auto& d = ray.direction;
            printf("KdTree accelerator test failure:\n"
                "Rays validated so far: %d (%.2f%%)\n"
                "KdTree T %.16g [%a]\n"
                "actual T %.16g [%a]\n"
                "ray origin: (%a, %a, %a)\n"
                "ray direction: (%a, %a, %a)\n",
                i, float(i) / float(ray_count),
                kdtree_hit_distance, kdtree_hit_distance,
                brute_force_intersection.t, brute_force_intersection.t,
                o.x, o.y, o.z, d.x, d.y, d.z
            );
            error("KdTree traversal error detected");
        }

        if (kdtree_hit_distance != Infinity) {
            last_hit = kdtree_intersection.position;
            last_hit_epsilon = kdtree_hit_distance * 1e-3f;
        }
    }

    printf("DONE\n");
}

const bool build_tree = false;

void test_kdtree() {
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    pcg32_random_t rng;
    pcg32_srandom_r(&rng, 0, 0);

    std::unique_ptr<Triangle_Mesh> mesh = LoadTriangleMesh(model_path);

    if (build_tree) {
        KdTree_Build_Params params;
        Timestamp t;
        Mesh_KdTree kdtree = build_kdtree(*mesh, params);
        int time = int(elapsed_milliseconds(t));
        printf("KdTree build time = %dms\n", time);
		kdtree.calculate_stats().print();

        kdtree.save_to_file("test.kdtree");
        printf("\n");
        return;
    }
    auto kdtree = load_mesh_kdtree(kdtree_path, *mesh);
    //auto kdtree = std::unique_ptr<KdTree>(new KdTree("test.kdtree", mesh));

    mesh->print_info();
    kdtree.calculate_stats().print();
    printf("\n");
    printf("=========================\n");
    printf("shooting rays (kdtree)...\n");

    int timeMsec = benchmark_kd_tree(kdtree);
    double speed = (benchmark_ray_count / 1000000.0) / (timeMsec / 1000.0);
    printf("raycast performance [%-6s]: %.2f MRays/sec, (rnd = %d)\n", model_path.c_str(), speed, pcg32_random_r(&rng));

    validate_kdtree(kdtree, validation_ray_count);
}
