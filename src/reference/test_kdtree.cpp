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
#include "kdtree_stats.h"
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

struct Operation_Info {
    std::string mesh_file_name;
    Triangle_Mesh* custom_mesh = nullptr;
    std::string custom_mesh_name;
    int validation_ray_count = 0;
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

static void benchmark_geometry_kdtree(const KdTree& kdtree, const Operation_Info&) {
    const bool debug_rays = false;
    const int debug_ray_count = 4;

    Vector3 last_hit_position = (kdtree.bounds.min_p + kdtree.bounds.max_p) * 0.5f;
    Vector3 last_hit_normal = Vector3(1, 0, 0);
    Ray_Generator ray_generator(kdtree.bounds);

    printf("shooting %.2gM rays against kdtree...\n", benchmark_ray_count / 1e6);

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
    printf("single raycast time: %.2f nanoseconds, %d clocks\n", nanoseconds_per_raycast, clocks);
    double mrays_per_sec = (benchmark_ray_count / 1e6f) / (time_ns / 1e9f);
    printf("raycast performance: %.2f MRays/sec\n\n", mrays_per_sec);
}

static void validate_triangle_mesh_kdtree(const KdTree& kdtree, const Operation_Info& info) {
    const Triangle_Mesh& mesh = *static_cast<const Triangle_Mesh_Geometry_Data*>(kdtree.geometry_data)->mesh;

    printf("Running triangle mesh kdtree validation... ");
    Vector3 last_hit_position = (kdtree.bounds.min_p + kdtree.bounds.max_p) * 0.5f;
    Vector3 last_hit_normal = Vector3(1, 0, 0);
    Ray_Generator ray_generator(kdtree.bounds);

    for (int i = 0; i < info.validation_ray_count; i++) {
        const Ray ray = ray_generator.generate_ray(last_hit_position, last_hit_normal);

        Intersection kdtree_intersection;
        kdtree.intersect(ray, kdtree_intersection);

        Intersection brute_force_intersection;
        int hit_k = -1; // for debugging
        for (int k = 0; k < mesh.get_triangle_count(); k++) {
            float ray_tmax = brute_force_intersection.t;
            kdtree.intersector(ray, kdtree.geometry_data, k, brute_force_intersection);
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
                i, float(i) / float(info.validation_ray_count),
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

static std::vector<Triangle_Mesh> create_custom_meshes() {
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

static void process_kdrees(std::function<void (const KdTree&, const Operation_Info&)> kdtree_handler)
{
    std::vector<Triangle_Mesh> custom_meshes = create_custom_meshes();

    std::vector<Operation_Info> infos{
        {"", &custom_meshes[0], "<mesh to test FP bug fix in clip_bounds function>", 100'000},
        {"../data/test-files/teapot.obj", nullptr, "", 100'000},
        {"../data/test-files/bunny.obj", nullptr, "", 10'000},
        {"../data/test-files/dragon.obj", nullptr, "", 5'000},
    };

    for (const Operation_Info& info : infos) {
        printf("================================================================\n");
        printf("Ray casting triangle mesh: %s\n", info.custom_mesh_name.empty()
            ? info.mesh_file_name.c_str()
            : info.custom_mesh_name.c_str());
        printf("================================================================\n");

        Triangle_Mesh mesh;
        if (!info.mesh_file_name.empty()) {
            Obj_Data obj_data = load_obj(info.mesh_file_name);
            ASSERT(!obj_data.meshes.empty());
            mesh = std::move(obj_data.meshes[0].mesh);
        }
        else {
            ASSERT(info.custom_mesh != nullptr);
            mesh = *info.custom_mesh;
        }
        printf("triangle count = %d\n", mesh.get_triangle_count());

        Triangle_Mesh_Geometry_Data geometry_data;
        geometry_data.mesh = &mesh;

        fs::path kdtree_filename = fs::path(info.mesh_file_name).replace_extension(".kdtree");
        if (!info.mesh_file_name.empty() && !fs_exists(kdtree_filename)) {
            Timestamp t;
            KdTree kdtree = build_triangle_mesh_kdtree(&geometry_data);
            printf("KdTree build time = %.2fs\n", elapsed_milliseconds(t) / 1000.f);
            kdtree.save(kdtree_filename.string());
            printf("\n");
            kdtree_calculate_stats(kdtree).print();
        }

        KdTree triangle_mesh_kdtree;
        if (!info.mesh_file_name.empty()) {
            triangle_mesh_kdtree = KdTree::load(kdtree_filename.string());
            triangle_mesh_kdtree.set_geometry_data(&geometry_data);
        }
        else {
            Timestamp t;
            triangle_mesh_kdtree = build_triangle_mesh_kdtree(&geometry_data);
            printf("KdTree build time = %.2fs\n", elapsed_milliseconds(t) / 1000.f);
        }
        kdtree_handler(triangle_mesh_kdtree, info);
    }
}

void test_kdtree()
{
    process_kdrees(&validate_triangle_mesh_kdtree);
}

void benchmark_kdtree()
{
    process_kdrees(&benchmark_geometry_kdtree);
}
