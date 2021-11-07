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
    Ray generate_ray(const Vector3& last_hit_position, const Vector3& last_hit_normal);

private:
    RNG rng;
    Bounding_Box ray_bounds;
};
} // namesapce

Ray_Generator::Ray_Generator(const Bounding_Box& mesh_bounds) {
    Vector3 diagonal = mesh_bounds.max_p - mesh_bounds.min_p;
    float delta = 2.0f * diagonal.length();
    Vector3 p_min = mesh_bounds.min_p - Vector3(delta);
    Vector3 p_max = mesh_bounds.max_p + Vector3(delta);
    ray_bounds = Bounding_Box(p_min, p_max);
}

Ray Ray_Generator::generate_ray(const Vector3& last_hit_position, const Vector3& last_hit_normal) {
    Vector3 origin;
    Vector3 direction;
    const bool use_last_hit = rng.get_float() < 0.25f;

    if (use_last_hit) {
        origin = last_hit_position;
        direction = sample_hemisphere_uniform(rng.get_vector2());
        Vector3 axis_a, axis_b;
        coordinate_system_from_vector(last_hit_normal, &axis_a, &axis_b);
        Matrix3x4 m = Matrix3x4::identity;
        m.set_column(0, axis_a);
        m.set_column(1, axis_b);
        m.set_column(2, last_hit_normal);
        direction = transform_vector(m, direction);
    }
    else {
        auto random_from_range = [this](float a, float b) {
            return a + (b - a) * rng.get_float();
        };
        origin.x = random_from_range(ray_bounds.min_p.x, ray_bounds.max_p.x);
        origin.y = random_from_range(ray_bounds.min_p.y, ray_bounds.max_p.y);
        origin.z = random_from_range(ray_bounds.min_p.z, ray_bounds.max_p.z);

        direction = sample_sphere_uniform(rng.get_vector2());
        if (rng.get_float() < 1.0f / 32.0f && direction.z != 0.0) {
            direction.x = direction.y = 0.0;
        }
        else if (rng.get_float() < 1.0f / 32.0f && direction.y != 0.0) {
            direction.x = direction.z = 0.0;
        }
        else if (rng.get_float() < 1.0f / 32.0f && direction.x != 0.0) {
            direction.y = direction.z = 0.0;
        }
        direction = direction.normalized();
    }
    return Ray{origin, direction};
}

const int benchmark_ray_count = 1'000'000;

static int benchmark_geometry_kdtree(const Geometry_KdTree& kdtree) {
    const bool debug_rays = false;
    const int debug_ray_count = 4;

    Bounding_Box bounds = kdtree.get_bounds();
    Vector3 last_hit_position = (bounds.min_p + bounds.max_p) * 0.5f;
    Vector3 last_hit_normal = Vector3(1, 0, 0);
    Ray_Generator ray_generator(bounds);

    int64_t time_ns = 0;
    for (int i = 0; i < benchmark_ray_count; i++) {
        const Ray ray = ray_generator.generate_ray(last_hit_position, last_hit_normal);

        Timestamp t;
        Intersection isect;
        bool hit_found = kdtree.intersect(ray, isect);
        time_ns += elapsed_nanoseconds(t);

        if (hit_found) {
            const Triangle_Intersection& ti = isect.triangle_intersection;
            Vector3 p = ti.mesh->get_position(ti.triangle_index, ti.barycentrics);

            Vector3 p0, p1, p2;
            ti.mesh->get_triangle(ti.triangle_index, p0, p1, p2);
            Vector3 ng = cross(p1 - p0, p2 - p0).normalized();
            ng = dot(ng, -ray.direction) < 0 ? -ng : ng;
            p = offset_ray_origin(p, ng);

            last_hit_position = p;
            last_hit_normal = ng;
        }

        if (debug_rays && i < debug_ray_count) {
            if (hit_found)
                printf("%d: found: %s, last_hit: %.14f %.14f %.14f\n", i,
                    hit_found ? "true" : "false",
                    last_hit_position.x, last_hit_position.y, last_hit_position.z);
            else
                printf("%d: found: %s\n", i, hit_found ? "true" : "false");
        }
    }

    double cpu_ghz = get_base_cpu_frequency_ghz();
    double nanoseconds_per_raycast = time_ns / double(benchmark_ray_count);
    int clocks = static_cast<int>(nanoseconds_per_raycast * cpu_ghz);
    printf("Single raycast time: %.2f nanoseconds, %d clocks\n", nanoseconds_per_raycast, clocks);
    return (int)(time_ns / 1'000'000);
}

static void validate_triangle_mesh_kdtree(const Geometry_KdTree& kdtree, int ray_count) {
    const Geometry_Primitive_Source& primitive_source = kdtree.get_primitive_source();
    ASSERT(primitive_source.geometry.type == Geometry_Type::triangle_mesh);

    printf("Running triangle mesh kdtree validation... ");
    auto bounds = kdtree.get_bounds();
    Vector3 last_hit_position = (bounds.min_p + bounds.max_p) * 0.5f;
    Vector3 last_hit_normal = Vector3(1, 0, 0);
    Ray_Generator ray_generator(bounds);

    for (int i = 0; i < ray_count; i++) {
        const Ray ray = ray_generator.generate_ray(last_hit_position, last_hit_normal);

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
            Vector3 p = ti.mesh->get_position(ti.triangle_index, ti.barycentrics);

            Vector3 p0, p1, p2;
            ti.mesh->get_triangle(ti.triangle_index, p0, p1, p2);
            Vector3 ng = cross(p1 - p0, p2 - p0).normalized();
            ng = dot(ng, -ray.direction) < 0 ? -ng : ng;
            p = offset_ray_origin(p, ng);

            last_hit_position = p;
            last_hit_normal = ng;
        }
    }
    printf("DONE\n");
}

namespace {
struct Triangle_Mesh_Info {
    std::string file_name;
    int validation_ray_count = 0;
    Triangle_Mesh* custom_mesh = nullptr;
};
}

static void test_triangle_mesh(const Triangle_Mesh_Info& triangle_mesh_info) {
    Geometries geometries;
    if (!triangle_mesh_info.file_name.empty()) {
        Obj_Data obj_data = load_obj(triangle_mesh_info.file_name);
        ASSERT(!obj_data.meshes.empty());
        geometries.triangle_meshes.push_back(std::move(obj_data.meshes[0].mesh));
    }
    else {
        ASSERT(triangle_mesh_info.custom_mesh != nullptr);
        geometries.triangle_meshes.push_back(*triangle_mesh_info.custom_mesh);
    }

    Timestamp t;
    KdTree_Build_Params params;
    //params.split_clipping = false;
    Geometry_KdTree triangle_mesh_kdtree = build_geometry_kdtree(&geometries, {Geometry_Type::triangle_mesh, 0}, params);
    printf("kdtree build time = %.2fs\n\n", elapsed_milliseconds(t) / 1000.f);
    triangle_mesh_kdtree.calculate_stats().print();

    printf("\nshooting rays (kdtree)...\n");
    int time_msec = benchmark_geometry_kdtree(triangle_mesh_kdtree);
    double speed = (benchmark_ray_count / 1000000.0) / (time_msec / 1000.0);
    printf("raycast performance [%-6s]: %.2f MRays/sec\n", triangle_mesh_info.file_name.c_str(), speed);

    validate_triangle_mesh_kdtree(triangle_mesh_kdtree, triangle_mesh_info.validation_ray_count);
}

std::vector<Triangle_Mesh> create_custom_meshes() {
    std::vector<Triangle_Mesh> meshes;
    // mesh 0
    {
        Triangle_Mesh mesh;
        mesh.vertices = {
            {-10.0000000f, -4.14615011f, -10.0000000f },
            {-10.0000000f, -4.14615011f,  10.0000000f },
            { 10.0000000f, -4.14615011f,  10.0000000f },
            { 10.0000000f, -4.14615011f, -10.0000000f },
            {-10.0000000f, -10.0000000f, -2.00000000f },
            { 10.0000000f, -10.0000000f, -2.00000000f },
            { 10.0000000f,  10.0000000f, -2.00000000f },
            {-10.0000000f,  10.0000000f, -2.00000000f }
        };
        mesh.indices = {
            0, 1, 2,
            2, 3, 0,
            4, 5, 6,
            6, 7, 4
        };
        meshes.push_back(std::move(mesh));
    }
    return meshes;
}

void test_kdtree() {
#ifdef _WIN32
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif

    std::vector<Triangle_Mesh> custom_meshes = create_custom_meshes();

    std::vector<Triangle_Mesh_Info> triangle_mesh_infos {
        { "", 100'000, &custom_meshes[0]},
        { "../data/test-files/teapot.obj", 100'000 },
        { "../data/test-files/bunny.obj", 10'000 },
        { "../data/test-files/dragon.obj", 5'000 },
    };

    for (const Triangle_Mesh_Info& info : triangle_mesh_infos) {
        printf("---------------------\n");
        printf("Triangle mesh: %s\n", info.file_name.c_str());
        test_triangle_mesh(info);
    }
 }
