#include "std.h"
#include "lib/common.h"
#include "reference_renderer.h"

#include "camera.h"
#include "direct_lighting.h"
#include "film.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "render_context.h"

#include "lib/geometry.h"
#include "lib/project.h"
#include "lib/random.h"
#include "lib/triangle_mesh.h"
#include "lib/vector.h"

#include "enkiTS/TaskScheduler.h"
#include "half/half.h"

constexpr int Tile_Size = 64;

// from third_party/miniexr.cpp
unsigned char* miniexr_write(unsigned width, unsigned height, unsigned channels, const void* rgba16f, size_t* out_size);

static void write_exr_image(const char* file_name, const ColorRGB* pixels, int w, int h) {
    FILE* file;
    if (fopen_s(&file, file_name, "wb") != 0)
        return;

    std::vector<unsigned short> rgb16f(w * h * 3);

    unsigned short* p = rgb16f.data();
    const ColorRGB* pixel = pixels;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            *p++ = float_to_half(pixel->r);
            *p++ = float_to_half(pixel->g);
            *p++ = float_to_half(pixel->b);
            pixel++;
        }
    }

    size_t exr_size;
    unsigned char* exr_data = miniexr_write(w, h, 3, rgb16f.data(), &exr_size);

    size_t bytes_written = fwrite(exr_data, 1, exr_size, file);
    ASSERT(bytes_written == exr_size);

    free(exr_data);
    fclose(file);
}

static fs::path get_kdtree_cache_path(const std::string& project_dir) {
    return get_data_dir_path() / project_dir / "kdtrees";
}

static bool create_kdtree_cache(const std::string& project_dir, const Geometries* geometries) {
    const fs::path kdtree_cache_dir = get_kdtree_cache_path(project_dir);

    if (fs_exists(kdtree_cache_dir) && !fs_remove_all(kdtree_cache_dir))
        return false;
    if (!fs_create_directory(kdtree_cache_dir))
        return false;

    for (int i = 0; i < (int)geometries->triangle_meshes.size(); i++) {
        Geometry_KdTree kdtree = build_geometry_kdtree(geometries, {Geometry_Type::triangle_mesh, i});
        fs::path kdtree_file = kdtree_cache_dir / (std::to_string(i) + ".kdtree");
        kdtree.save_to_file(kdtree_file.string());
    }
    return true;
}

static std::vector<Geometry_KdTree> load_kdtree_cache(const std::string& project_dir, const Geometries* geometries) {
    const fs::path kdtree_cache_dir = get_kdtree_cache_path(project_dir);

    std::vector<Geometry_KdTree> kdtrees(geometries->triangle_meshes.size());
    for (int i = 0; i < (int)geometries->triangle_meshes.size(); i++) {
        fs::path kdtree_file = kdtree_cache_dir / (std::to_string(i) + ".kdtree");
        kdtrees[i] = load_geometry_kdtree(kdtree_file.string(), geometries, {Geometry_Type::triangle_mesh, i});
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
            ColorRGB radiance = compute_direct_lighting(ctx, local_geom, wo, local_geom.material, &rng);
            tile.add_sample(film_pos, radiance);
        }
    }
    film.merge_tile(tile);
}

void render_reference_image(const YAR_Project& project, const Renderer_Options& options) {
    Scene scene = load_project(project);

    printf("Preparing meshes\n");
    for (auto [i, light] : enumerate(scene.lights.diffuse_rectangular_lights)) {
        scene.geometries.triangle_meshes.emplace_back(light.get_geometry());
    }

    if (!fs_exists(get_kdtree_cache_path(scene.project_dir))) {
        printf("Creating kdtree cache...\n");
        Timestamp t;
        if (!create_kdtree_cache(scene.project_dir, &scene.geometries)) {
            printf("failed to create kdtree cache\n");
            return;
        }
        printf("KdTree cache build time = %.2f s\n", elapsed_milliseconds(t) / 1e3f);
    }
    std::vector<Geometry_KdTree> kdtrees = load_kdtree_cache(scene.project_dir, &scene.geometries);

    Scene_KdTree scene_kdtree = build_scene_kdtree(&scene, kdtrees);
    printf("scene tree created\n");

    Matrix3x4 camera_to_world = project.camera_to_world;
    for (int i = 0; i < 3; i++) {
        float tmp = camera_to_world.a[i][1];
        camera_to_world.a[i][1] = -camera_to_world.a[i][2];
        camera_to_world.a[i][2] = tmp;
    }
    Camera camera(camera_to_world, Vector2(project.image_resolution), 60.f);

    Bounds2i render_region = project.render_region;
    if (render_region == Bounds2i{})
        render_region = { {0, 0}, project.image_resolution }; // default render region

    ASSERT(render_region.p0 >= Vector2i{});
    ASSERT(render_region.p1 <= project.image_resolution);
    ASSERT(render_region.p0 < render_region.p1);

    const float filter_radius = 0.5f;

    Film film(render_region.size(), render_region, get_box_filter(filter_radius));

    Bounds2i sample_region {
        Vector2i {
            (int32_t)std::ceil(render_region.p0.x + 0.5f - filter_radius),
            (int32_t)std::ceil(render_region.p0.y + 0.5f - filter_radius)
        },
        Vector2i {
            (int32_t)std::ceil(render_region.p1.x-1 + 0.5f + filter_radius),
            (int32_t)std::ceil(render_region.p1.y-1 + 0.5f + filter_radius)
        } + Vector2i{1, 1}
    };

    Vector2i sample_region_size = sample_region.p1 - sample_region.p0;

    const int x_tile_count = (sample_region_size.x + Tile_Size - 1) / Tile_Size;
    const int y_tile_count = (sample_region_size.y + Tile_Size - 1) / Tile_Size;

    Render_Context ctx;
    ctx.camera = &camera;
    ctx.acceleration_structure = &scene_kdtree;
    ctx.lights = scene.lights;
    ctx.materials = scene.materials;

    Timestamp t;

    if (options.cpu_core_count == 1) {
        for (int y_tile = 0; y_tile < y_tile_count; y_tile++) {
            for (int x_tile = 0; x_tile < x_tile_count; x_tile++) {
                Bounds2i tile_sample_bounds;
                tile_sample_bounds.p0 = sample_region.p0 + Vector2i{ x_tile * Tile_Size, y_tile * Tile_Size };
                tile_sample_bounds.p1.x = std::min(tile_sample_bounds.p0.x + Tile_Size, sample_region.p1.x);
                tile_sample_bounds.p1.y = std::min(tile_sample_bounds.p0.y + Tile_Size, sample_region.p1.y);

                Bounds2i tile_pixel_bounds;
                tile_pixel_bounds.p0.x = std::max((int)std::ceil(tile_sample_bounds.p0.x + 0.5f - filter_radius), render_region.p0.x);
                tile_pixel_bounds.p0.y = std::max((int)std::ceil(tile_sample_bounds.p0.y + 0.5f - filter_radius), render_region.p0.y);
                tile_pixel_bounds.p1.x = std::min((int)std::ceil(tile_sample_bounds.p1.x - 1 + 0.5f + filter_radius) + 1, render_region.p1.x);
                tile_pixel_bounds.p1.y = std::min((int)std::ceil(tile_sample_bounds.p1.y - 1 + 0.5f + filter_radius) + 1, render_region.p1.y);

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
                tile_pixel_bounds.p0.x = std::max((int)std::ceil(tile_sample_bounds.p0.x + 0.5f - filter_radius), render_region.p0.x);
                tile_pixel_bounds.p0.y = std::max((int)std::ceil(tile_sample_bounds.p0.y + 0.5f - filter_radius), render_region.p0.y);
                tile_pixel_bounds.p1.x = std::min((int)std::ceil(tile_sample_bounds.p1.x - 1 + 0.5f + filter_radius) + 1, render_region.p1.x);
                tile_pixel_bounds.p1.y = std::min((int)std::ceil(tile_sample_bounds.p1.y - 1 + 0.5f + filter_radius) + 1, render_region.p1.y);

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

    write_exr_image("image.exr",  image.data(), render_region.size().x, render_region.size().y);
}
