#include "std.h"
#include "lib/common.h"
#include "lib/obj_loader.h"
#include "lib/vector.h"
#include "../intersection.h"
#include "../kdtree.h"
#include "../kdtree_builder.h"
#include "../sampling.h"
#include "../triangle_mesh.h"

#ifdef _WIN32
#include <pmmintrin.h>
#endif

class Ray_Generator {
public:
    Ray_Generator(const Bounding_Box& mesh_bounds);
    Ray generate_ray(const Vector3& last_hit, float last_hit_epsilon);

private:
    pcg32_random_t rng;
    Bounding_Box ray_bounds;
};

Ray_Generator::Ray_Generator(const Bounding_Box& mesh_bounds) {
    pcg32_srandom_r(&rng, 0, 0);

    auto diagonal = mesh_bounds.max_p - mesh_bounds.min_p;
    float delta = 2.0f * diagonal.length();

    auto p_min = mesh_bounds.min_p - Vector3(delta);
    auto p_max = mesh_bounds.max_p + Vector3(delta);
    ray_bounds = Bounding_Box(p_min, p_max);
}

Ray Ray_Generator::generate_ray(const Vector3& last_hit, float last_hit_epsilon) {
    auto random_from_range = [this](float a, float b) {
        return a + (b - a) * random_float(&rng);
    };

    // Ray origin.
    Vector3 origin;
    origin.x = random_from_range(ray_bounds.min_p.x, ray_bounds.max_p.x);
    origin.y = random_from_range(ray_bounds.min_p.y, ray_bounds.max_p.y);
    origin.z = random_from_range(ray_bounds.min_p.z, ray_bounds.max_p.z);

    const bool use_last_hit = random_float(&rng) < 0.25f;
    if (use_last_hit)
        origin = last_hit;

    // Ray direction.
    auto direction = uniform_sample_sphere(random_float(&rng), random_float(&rng));
    auto len = direction.length();

    if (random_float(&rng) < 1.0f / 32.0f && direction.z != 0.0)
        direction.x = direction.y = 0.0;
    else if (random_float(&rng) < 1.0f / 32.0f && direction.y != 0.0)
        direction.x = direction.z = 0.0;
    else if (random_float(&rng) < 1.0f / 32.0f && direction.x != 0.0)
        direction.y = direction.z = 0.0;
    direction = direction.normalized();

    auto ray = Ray(origin, direction);
    ray.origin = ray.get_point(use_last_hit ? last_hit_epsilon : 1e-3f);
    return ray;
}

const int benchmark_ray_count = 1'000'000;

struct Model_Info {
    std::string file_name;
    int validation_ray_count;
};

static std::vector<Model_Info> model_infos {
    { "test-files/teapot.obj", 100'000 },
    { "test-files/bunny.obj", 10'000 },
    { "test-files/dragon.obj", 5'000 },
};

int benchmark_mesh_kdtree(const Mesh_KdTree& kdtree) {
    const bool debug_rays = false;
    const int debug_ray_count = 4;

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

    double cpu_ghz = get_base_cpu_frequency_ghz();
    double nanoseconds_per_raycast = time_ns / double(benchmark_ray_count);
    int clocks = static_cast<int>(nanoseconds_per_raycast * cpu_ghz);
    printf("Single raycast time: %.2f nanoseconds, %d clocks\n", nanoseconds_per_raycast, clocks);

    //return static_cast<int>(elapsed_milliseconds(t));
    return (int)(time_ns / 1'000'000);
}

void validate_mesh_kdtree(const Mesh_KdTree& kdtree, int ray_count) {
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

TwoLevel_KdTree get_toplevel_kdtree(const std::string& model_file_name, const std::vector<Triangle_Mesh>& model_meshes, std::vector<Mesh_KdTree>& mesh_kdtrees) {
    // Build kdtrees for all meshes.
    Timestamp t;
    KdTree_Build_Params build_params;
    mesh_kdtrees.resize(model_meshes.size());
    for (auto [i, mesh] : enumerate(model_meshes)) {
        mesh_kdtrees[i] = build_kdtree(mesh, build_params);
        printf("kdtree %d\n", static_cast<int>(i));
        mesh_kdtrees[i].calculate_stats().print();
    }
    printf("Mesh kdtrees build time = %.2fs\n", elapsed_milliseconds(t) / 1000.f);

    // Build top-level kdtree.
    Timestamp t2;
    TwoLevel_KdTree top_level_kdtree = build_kdtree(mesh_kdtrees, build_params);
    printf("top-level kdtree\n");
    top_level_kdtree.calculate_stats().print();
    printf("Top level kdtree build time = %.2fs\n", elapsed_milliseconds(t2) / 1000.f);
    return top_level_kdtree;
}

void test_model(const Model_Info& model_info) {
    std::vector<Triangle_Mesh> meshes;
    {
        std::vector<Obj_Model> obj_models = load_obj(model_info.file_name);
        for (const Obj_Model& obj_model : obj_models) {
            meshes.push_back(Triangle_Mesh::from_mesh_data(obj_model.mesh_data));
        }
    }

    Timestamp t;
    KdTree_Build_Params build_params;
    Mesh_KdTree mesh_kdtree = build_kdtree(meshes[0], build_params);
    printf("kdtree build time = %.2fs\n\n", elapsed_milliseconds(t) / 1000.f);
    mesh_kdtree.calculate_stats().print();

    printf("\nshooting rays (kdtree)...\n");
    int time_msec = benchmark_mesh_kdtree(mesh_kdtree);
    double speed = (benchmark_ray_count / 1000000.0) / (time_msec / 1000.0);
    printf("raycast performance [%-6s]: %.2f MRays/sec\n", model_info.file_name.c_str(), speed);

    validate_mesh_kdtree(mesh_kdtree, model_info.validation_ray_count);
}

void test_kdtree() {
#ifdef _WIN32
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif

    for (const Model_Info& model_info : model_infos) {
        printf("---------------------\n");
        printf("Model: %s\n", model_info.file_name.c_str());
        test_model(model_info);
    }
 }
