#include "std.h"
#include "reference_renderer.h"

#include "camera.h"
#include "film.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "light.h"
#include "triangle_mesh.h"

#include "lib/io.h"
#include "lib/common.h"
#include "lib/geometry.h"
#include "lib/mesh.h"
#include "lib/random.h"
#include "lib/vector.h"

#include "enkiTS/TaskScheduler.h"

namespace {
struct Render_Context {
    Bounds2i sample_bounds;
    const Camera* camera;
    const TwoLevel_KdTree* acceleration_structure;
    Lights lights;
};
} // namespace

constexpr int Tile_Size = 64;

static fs::path get_kdtree_cache_path(const std::string& project_dir) {
    return get_data_dir_path() / project_dir / "kdtrees";
}

static bool create_kdtree_cache(const std::string& project_dir, const std::vector<Triangle_Mesh>& meshes) {
    const fs::path kdtree_cache_dir = get_kdtree_cache_path(project_dir);

    if (fs_exists(kdtree_cache_dir) && !fs_remove_all(kdtree_cache_dir))
        return false;
    if (!fs_create_directory(kdtree_cache_dir))
        return false;

    for (int i = 0; i < (int)meshes.size(); i++) {
        Mesh_KdTree kdtree = build_kdtree(meshes[i]);
        fs::path kdtree_file = kdtree_cache_dir / (std::to_string(i) + ".kdtree");
        kdtree.save_to_file(kdtree_file.string());
    }
    return true;
}

static std::vector<Mesh_KdTree> load_kdtree_cache(const std::string& project_dir, const std::vector<Triangle_Mesh>& meshes) {
    const fs::path kdtree_cache_dir = get_kdtree_cache_path(project_dir);

    std::vector<Mesh_KdTree> kdtrees(meshes.size());
    for (int i = 0; i < (int)meshes.size(); i++) {
        fs::path kdtree_file = kdtree_cache_dir / (std::to_string(i) + ".kdtree");
        kdtrees[i] = load_mesh_kdtree(kdtree_file.string(), meshes[i]);
    }
    return kdtrees;
}

static void render_tile(const Render_Context& ctx, Bounds2i sample_bounds, Bounds2i pixel_bounds, uint64_t rng_seed, Film& film) {
    pcg32_random_t rng;
    pcg32_srandom_r(&rng, 0, rng_seed);

    Film_Tile tile(pixel_bounds, film.filter);

    for (int y = sample_bounds.p0.y; y < sample_bounds.p1.y; y++) {
        for (int x = sample_bounds.p0.x; x < sample_bounds.p1.x; x++) {
            Vector2 film_pos = Vector2(x + 0.5f, y + 0.5f);

            Ray ray = ctx.camera->generate_ray(film_pos);

            Local_Geometry local_geom;
            if (ctx.acceleration_structure->intersect(ray, local_geom) == Infinity)
                continue;

            Vector3 wo = (ray.origin - local_geom.position).normalized();
            ColorRGB radiance = compute_direct_lighting(local_geom, ctx.acceleration_structure, ctx.lights, wo, local_geom.material, &rng);
            tile.add_sample(film_pos, radiance);
        }
    }
    film.merge_tile(tile);
}

void render_reference_image(const Reference_Renderer_Input& input) {
    printf("Preparing meshes\n");
    std::vector<Triangle_Mesh> meshes(input.scene_data->meshes.size());
    for (size_t i = 0; i < input.scene_data->meshes.size(); i++) {
        Material_Handle material_handle = register_material(input.scene_data->materials[i]);
        meshes[i] = Triangle_Mesh::from_mesh_data(input.scene_data->meshes[i], material_handle);
    }

    if (!fs_exists(get_kdtree_cache_path(input.scene_data->project_dir))) {
        printf("Creating kdtree cache...\n");
        Timestamp t;
        if (!create_kdtree_cache(input.scene_data->project_dir, meshes)) {
            printf("failed to create kdtree cache\n");
            return;
        }
        printf("KdTree cache build time = %.2f s\n", elapsed_milliseconds(t) / 1e3f);
    }
    std::vector<Mesh_KdTree> kdtrees = load_kdtree_cache(input.scene_data->project_dir, meshes);

    TwoLevel_KdTree kdtree = build_kdtree(kdtrees);
    printf("two-level tree created\n");

    Matrix3x4 camera_to_world = input.camera_to_world;
    for (int i = 0; i < 3; i++) {
        float tmp = camera_to_world.a[i][1];
        camera_to_world.a[i][1] = -camera_to_world.a[i][2];
        camera_to_world.a[i][2] = tmp;
    }
    Camera camera(camera_to_world, Vector2(input.image_resolution), 60.f);

    Lights lights;
    {
        for (const RGB_Point_Light_Data& point_light_data : input.scene_data->rgb_point_lights) {
            Point_Light light;
            light.position = point_light_data.position;
            light.intensity = point_light_data.intensity;
            lights.point_lights.push_back(light);
        }
        for (const RGB_Diffuse_Rectangular_Light_Data& light_data : input.scene_data->rgb_diffuse_rectangular_lights) {
            lights.diffuse_rectangular_lights.push_back(Diffuse_Rectangular_Light(light_data));
        }
    }

    ASSERT(input.render_region.p0 >= Vector2i{});
    ASSERT(input.render_region.p1 <= input.image_resolution);
    ASSERT(input.render_region.p0 < input.render_region.p1);

    const float filter_radius = 0.5f;

    Film film(input.render_region.size(), input.render_region, get_box_filter(filter_radius));

    Bounds2i sample_region {
        Vector2i {
            (int32_t)std::ceil(input.render_region.p0.x + 0.5f - filter_radius),
            (int32_t)std::ceil(input.render_region.p0.y + 0.5f - filter_radius)
        },
        Vector2i {
            (int32_t)std::ceil(input.render_region.p1.x-1 + 0.5f + filter_radius),
            (int32_t)std::ceil(input.render_region.p1.y-1 + 0.5f + filter_radius)
        } + Vector2i{1, 1}
    };

    Vector2i sample_region_size = sample_region.p1 - sample_region.p0;

    const int x_tile_count = (sample_region_size.x + Tile_Size - 1) / Tile_Size;
    const int y_tile_count = (sample_region_size.y + Tile_Size - 1) / Tile_Size;

    Render_Context ctx;
    ctx.camera = &camera;
    ctx.acceleration_structure = &kdtree;
    ctx.lights = lights;

    Timestamp t;

    if (!input.parallel_rendering) {
        for (int y_tile = 0; y_tile < y_tile_count; y_tile++) {
            for (int x_tile = 0; x_tile < x_tile_count; x_tile++) {
                Bounds2i tile_sample_bounds;
                tile_sample_bounds.p0 = sample_region.p0 + Vector2i{ x_tile * Tile_Size, y_tile * Tile_Size };
                tile_sample_bounds.p1.x = std::min(tile_sample_bounds.p0.x + Tile_Size, sample_region.p1.x);
                tile_sample_bounds.p1.y = std::min(tile_sample_bounds.p0.y + Tile_Size, sample_region.p1.y);

                Bounds2i tile_pixel_bounds;
                tile_pixel_bounds.p0.x = std::max((int)std::ceil(tile_sample_bounds.p0.x + 0.5f - filter_radius), input.render_region.p0.x);
                tile_pixel_bounds.p0.y = std::max((int)std::ceil(tile_sample_bounds.p0.y + 0.5f - filter_radius), input.render_region.p0.y);
                tile_pixel_bounds.p1.x = std::min((int)std::ceil(tile_sample_bounds.p1.x - 1 + 0.5f + filter_radius) + 1, input.render_region.p1.x);
                tile_pixel_bounds.p1.y = std::min((int)std::ceil(tile_sample_bounds.p1.y - 1 + 0.5f + filter_radius) + 1, input.render_region.p1.y);

                uint64_t rng_seed = y_tile * x_tile_count + x_tile;
                render_tile(ctx, tile_sample_bounds, tile_pixel_bounds, rng_seed, film);
            }
        }
    } else {
        struct Render_Tile_Task : public enki::ITaskSet {
            Render_Context* ctx;
            Bounds2i tile_sample_bounds;
            Bounds2i tile_pixel_bounds;
            uint64_t rng_seed;
            Film* film;

            void ExecuteRange(enki::TaskSetPartition, uint32_t) override {
                render_tile(*ctx, tile_sample_bounds, tile_pixel_bounds, rng_seed, *film);
            }
        };

        std::vector<Render_Tile_Task> tasks;
        for (int y_tile = 0; y_tile < y_tile_count; y_tile++) {
            for (int x_tile = 0; x_tile < x_tile_count; x_tile++) {
                Bounds2i tile_sample_bounds;
                tile_sample_bounds.p0 = sample_region.p0 + Vector2i{ x_tile * Tile_Size, y_tile * Tile_Size };
                tile_sample_bounds.p1.x = std::min(tile_sample_bounds.p0.x + Tile_Size, sample_region.p1.x);
                tile_sample_bounds.p1.y = std::min(tile_sample_bounds.p0.y + Tile_Size, sample_region.p1.y);

                Bounds2i tile_pixel_bounds;
                tile_pixel_bounds.p0.x = std::max((int)std::ceil(tile_sample_bounds.p0.x + 0.5f - filter_radius), input.render_region.p0.x);
                tile_pixel_bounds.p0.y = std::max((int)std::ceil(tile_sample_bounds.p0.y + 0.5f - filter_radius), input.render_region.p0.y);
                tile_pixel_bounds.p1.x = std::min((int)std::ceil(tile_sample_bounds.p1.x - 1 + 0.5f + filter_radius) + 1, input.render_region.p1.x);
                tile_pixel_bounds.p1.y = std::min((int)std::ceil(tile_sample_bounds.p1.y - 1 + 0.5f + filter_radius) + 1, input.render_region.p1.y);

                uint64_t rng_seed = y_tile * x_tile_count + x_tile;

                Render_Tile_Task task{};
                task.ctx = &ctx;
                task.tile_sample_bounds = tile_sample_bounds;
                task.tile_pixel_bounds = tile_pixel_bounds;
                task.rng_seed = rng_seed;
                task.film = &film;
                tasks.push_back(task);
            }
        }

        enki::TaskScheduler task_scheduler;
        task_scheduler.Initialize();
        for (Render_Tile_Task& task : tasks) {
            task_scheduler.AddTaskSetToPipe(&task);
        }
        task_scheduler.WaitforAllAndShutdown();
    }

    std::vector<ColorRGB> image = film.get_image();

    int time = int(elapsed_milliseconds(t));
    printf("image rendered in %d ms\n", time);

    write_exr_image("image.exr",  image.data(), (int)input.render_region.size().x, (int)input.render_region.size().y);
}
