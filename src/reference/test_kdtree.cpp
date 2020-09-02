#include "std.h"
#include "lib/common.h"
#include "lib/math.h"
#include "lib/obj_loader.h"
#include "lib/random.h"
#include "lib/triangle_mesh.h"
#include "lib/vector.h"

#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "sampling.h"

#ifdef _WIN32
#include <pmmintrin.h>
#endif

namespace {
class Ray_Generator {
public:
    Ray_Generator(const Bounding_Box& mesh_bounds);
    Ray generate_ray(const Vector3& last_hit, float last_hit_epsilon);

private:
    pcg32_random_t rng;
    Bounding_Box ray_bounds;
};
} // namesapce

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
    auto direction = sample_sphere_uniform(Vector2(random_float(&rng), random_float(&rng)));
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

struct Triangle_Mesh_Info {
    std::string file_name;
    int validation_ray_count = 0;
};

static std::vector<Triangle_Mesh_Info> triangle_mesh_infos {
    { "../data/test-files/teapot.obj", 100'000 },
    { "../data/test-files/bunny.obj", 10'000 },
    { "../data/test-files/dragon.obj", 5'000 },
};

static int benchmark_geometry_kdtree(const Geometry_KdTree& kdtree) {
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
        Intersection isect;
        bool hit_found = kdtree.intersect(ray, isect);
        time_ns += elapsed_nanoseconds(t2);

        if (hit_found) {
            const Triangle_Intersection& ti = isect.triangle_intersection;
            Vector3 p = ti.mesh->get_position(ti.triangle_index, ti.b1, ti.b2);

            Vector3 p0, p1, p2;
            ti.mesh->get_triangle(ti.triangle_index, p0, p1, p2);
            Vector3 ng = cross(p1 - p0, p2 - p0).normalized();
            ng = dot(ng, -ray.direction) < 0 ? -ng : ng;
            p = offset_ray_origin(p, ng);

            last_hit = p;
            last_hit_epsilon = isect.t * 1e-3f;
        }

        if (debug_rays && i < debug_ray_count) {
            if (hit_found)
                printf("%d: found: %s, last_hit: %.14f %.14f %.14f\n", i,
                    hit_found ? "true" : "false", last_hit.x, last_hit.y, last_hit.z);
            else
                printf("%d: found: %s\n", i, hit_found ? "true" : "false");
        }
    }

    double cpu_ghz = get_base_cpu_frequency_ghz();
    double nanoseconds_per_raycast = time_ns / double(benchmark_ray_count);
    int clocks = static_cast<int>(nanoseconds_per_raycast * cpu_ghz);
    printf("Single raycast time: %.2f nanoseconds, %d clocks\n", nanoseconds_per_raycast, clocks);

    //return static_cast<int>(elapsed_milliseconds(t));
    return (int)(time_ns / 1'000'000);
}

static void validate_triangle_mesh_kdtree(const Geometry_KdTree& kdtree, int ray_count) {
    const Geometry_Primitive_Source& primitive_source = kdtree.get_primitive_source();
    ASSERT(primitive_source.geometry.type == Geometry_Type::triangle_mesh);

    printf("Running triangle mesh kdtree validation... ");
    auto bounds = kdtree.get_bounds();
    Vector3 last_hit = (bounds.min_p + bounds.max_p) * 0.5f;
    float last_hit_epsilon = 0.0f;

    auto ray_generator = Ray_Generator(bounds);

    for (int i = 0; i < ray_count; i++) {
        const Ray ray = ray_generator.generate_ray(last_hit, last_hit_epsilon);

        Intersection kdtree_intersection;
        kdtree.intersect(ray, kdtree_intersection);

        Intersection brute_force_intersection;
        int hit_k = -1; // for debugging
        for (int k = 0; k < primitive_source.get_primitive_count(); k++) {
            float ray_tmax = brute_force_intersection.t;
            intersect_geometric_primitive(ray, primitive_source.geometries, primitive_source.geometry, k, brute_force_intersection);
            if (brute_force_intersection.t < ray_tmax) {
                hit_k = k;
            }
        }

        if (kdtree_intersection.t != brute_force_intersection.t) {
            const auto& o = ray.origin;
            const auto& d = ray.direction;
            printf("KdTree accelerator test failure:\n"
                "Rays validated so far: %d (%.2f%%)\n"
                "KdTree T %.16g [%a]\n"
                "actual T %.16g [%a]\n"
                "ray origin: (%a, %a, %a)\n"
                "ray direction: (%a, %a, %a)\n",
                i, float(i) / float(ray_count),
                kdtree_intersection.t, kdtree_intersection.t,
                brute_force_intersection.t, brute_force_intersection.t,
                o.x, o.y, o.z, d.x, d.y, d.z
            );
            error("KdTree traversal error detected");
        }

        if (kdtree_intersection.t != Infinity) {
            const Triangle_Intersection& ti = kdtree_intersection.triangle_intersection;
            Vector3 p = ti.mesh->get_position(ti.triangle_index, ti.b1, ti.b2);

            Vector3 p0, p1, p2;
            ti.mesh->get_triangle(ti.triangle_index, p0, p1, p2);
            Vector3 ng = cross(p1 - p0, p2 - p0).normalized();
            ng = dot(ng, -ray.direction) < 0 ? -ng : ng;
            p = offset_ray_origin(p, ng);

            last_hit = p;
            last_hit_epsilon = kdtree_intersection.t * 1e-3f;
        }
    }

    printf("DONE\n");
}

static void test_triangle_mesh(const Triangle_Mesh_Info& triangle_mesh_info) {
    Obj_Data obj_data = load_obj(triangle_mesh_info.file_name);
    ASSERT(!obj_data.meshes.empty());

    Geometries geometries;
    geometries.triangle_meshes.push_back(std::move(obj_data.meshes[0].mesh));

    Timestamp t;
    Geometry_KdTree triangle_mesh_kdtree = build_geometry_kdtree(&geometries, {Geometry_Type::triangle_mesh, 0});
    printf("kdtree build time = %.2fs\n\n", elapsed_milliseconds(t) / 1000.f);
    triangle_mesh_kdtree.calculate_stats().print();

    printf("\nshooting rays (kdtree)...\n");
    int time_msec = benchmark_geometry_kdtree(triangle_mesh_kdtree);
    double speed = (benchmark_ray_count / 1000000.0) / (time_msec / 1000.0);
    printf("raycast performance [%-6s]: %.2f MRays/sec\n", triangle_mesh_info.file_name.c_str(), speed);

    validate_triangle_mesh_kdtree(triangle_mesh_kdtree, triangle_mesh_info.validation_ray_count);
}

void test_kdtree() {
#ifdef _WIN32
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif

    for (const Triangle_Mesh_Info& info : triangle_mesh_infos) {
        printf("---------------------\n");
        printf("Triangle mesh: %s\n", info.file_name.c_str());
        test_triangle_mesh(info);
    }
 }

