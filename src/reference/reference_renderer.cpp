#include "std.h"
#include "lib/common.h"
#include "reference_renderer.h"

#include "bsdf.h"
#include "camera.h"
#include "context.h"
#include "direct_lighting.h"
#include "film.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "shading_context.h"

#include "lib/geometry.h"
#include "lib/math.h"
#include "lib/random.h"
#include "lib/scene_loader.h"
#include "lib/triangle_mesh.h"
#include "lib/vector.h"

#include "enkiTS/TaskScheduler.h"
#include "half/half.h"
#include "meow-hash/meow_hash_x64_aesni.h"

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

// Returns a name that can be used to create a directory to store additional/generated project data.
// The name is based on the hash of the scene's full path. So, for different project files that
// reference the same scene this function will return the same string.
//
// NOTE: if per project temp directories are needed then one option is to create project
// specific subdirectories inside temp scene directory - in this case we can share 
// scene's additional data between multiple projects.
static std::string get_project_unique_name(const std::string& scene_path) {
    std::string file_name = to_lower(fs::path(scene_path).filename().string());
    if (file_name.empty())
        error("Failed to extract filename from scene path: %s", scene_path.c_str());

    std::string path_lowercase = to_lower(scene_path);
    meow_u128 hash_128 = MeowHash(MeowDefaultSeed, path_lowercase.size(), (void*)path_lowercase.c_str());
    uint32_t hash_32 = MeowU32From(hash_128, 0);

    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(8) << std::hex << hash_32;
    oss << "-" << file_name;
    return oss.str();
}

static Scene_KdTree load_scene_kdtree(const Scene& scene) {
    fs::path kdtree_cache_directory = get_data_directory() / "kdtree-cache" / get_project_unique_name(scene.path);

    // If kdtrees are not cached then build a cache.
    if (!fs_exists(kdtree_cache_directory)) {
        Timestamp t;
        printf("Kdtree cache is not found. Building kdtree cache: ");
        
        if (!fs_create_directories(kdtree_cache_directory))
            error("Failed to create kdtree cache directory: %s\n", kdtree_cache_directory.string().c_str());

        for (int i = 0; i < (int)scene.geometries.triangle_meshes.size(); i++) {
            Geometry_KdTree kdtree = build_geometry_kdtree(&scene.geometries, {Geometry_Type::triangle_mesh, i});
            fs::path kdtree_file = kdtree_cache_directory / (std::to_string(i) + ".kdtree");
            kdtree.save_to_file(kdtree_file.string());
        }
        printf("%.2fs\n", elapsed_milliseconds(t) / 1e3f);
    }

    // Load kdtrees from the cache.
    printf("Loading kdtree cache: ");
    std::vector<Geometry_KdTree> kdtrees(scene.geometries.triangle_meshes.size());
    for (int i = 0; i < (int)scene.geometries.triangle_meshes.size(); i++) {
        fs::path kdtree_file = kdtree_cache_directory / (std::to_string(i) + ".kdtree");
        kdtrees[i] = load_geometry_kdtree(kdtree_file.string(), &scene.geometries, {Geometry_Type::triangle_mesh, i});
    }
    printf("done\n");

    // Build top-level scene kdtree.
    Scene_KdTree scene_kdtree;
    {
        Timestamp t;
        printf("Building scene kdtree: ");
        scene_kdtree = build_scene_kdtree(&scene, std::move(kdtrees));
        printf("%.2fs\n", elapsed_milliseconds(t) / 1e3f);
    }
    return scene_kdtree;
}

static void init_pixel_sampler_config(Stratified_Pixel_Sampler_Configuration& pixel_sampler_config, Scene_Context& scene_ctx) {
    pixel_sampler_config.init(scene_ctx.scene->x_pixel_samples, scene_ctx.scene->y_pixel_samples);

    scene_ctx.array2d_registry.rectangular_light_arrays.reserve(scene_ctx.lights.diffuse_rectangular_lights.size());
    for (const Diffuse_Rectangular_Light& light : scene_ctx.lights.diffuse_rectangular_lights) {
        int k = (int)std::ceil(std::sqrt(light.sample_count));
        ASSERT(k*k >= light.sample_count);
        ASSERT((k-1)*(k-1) < light.sample_count);

        MIS_Array_Info info;
        info.light_array_id = pixel_sampler_config.register_array2d_samples(k, k);
        info.bsdf_array_id = pixel_sampler_config.register_array2d_samples(k, k);
        info.array_size = k*k;
        scene_ctx.array2d_registry.rectangular_light_arrays.push_back(info);
    }

    scene_ctx.array2d_registry.sphere_light_arrays.reserve(scene_ctx.lights.diffuse_sphere_lights.size());
    for (const Diffuse_Sphere_Light& light : scene_ctx.lights.diffuse_sphere_lights) {
        int k = (int)std::ceil(std::sqrt(light.sample_count));
        ASSERT(k*k >= light.sample_count);
        ASSERT((k-1)*(k-1) < light.sample_count);

        MIS_Array_Info info;
        info.light_array_id = pixel_sampler_config.register_array2d_samples(k, k);
        info.bsdf_array_id = pixel_sampler_config.register_array2d_samples(k, k);
        info.array_size = k*k;
        scene_ctx.array2d_registry.sphere_light_arrays.push_back(info);
    }
}

static void render_tile(const Scene_Context& ctx, Thread_Context& thread_ctx, Bounds2i sample_bounds, Bounds2i pixel_bounds, uint64_t rng_seed, Film& film) {
    thread_ctx.rng.init(0, rng_seed);
    Film_Tile tile(pixel_bounds, film.filter);

    for (int y = sample_bounds.p0.y; y < sample_bounds.p1.y; y++) {
        for (int x = sample_bounds.p0.x; x < sample_bounds.p1.x; x++) {
            thread_ctx.pixel_sampler.generate_samples(thread_ctx.rng);

            for (int s = 0; s < thread_ctx.pixel_sampler.get_pixel_sample_count(); s++) {
                thread_ctx.memory_pool.reset();
                thread_ctx.current_pixel_sample_index = s;

                Vector2 film_pos = Vector2((float)x, (float)y) + thread_ctx.pixel_sampler.get_image_plane_position(s);
                Ray ray = ctx.camera->generate_ray(film_pos);

                Intersection isect;
                if (ctx.acceleration_structure->intersect(ray, isect)) {
                    Shading_Point_Rays rays;
                    rays.incident_ray = ray;
                    rays.auxilary_ray_dx_offset = ctx.camera->generate_ray(Vector2(film_pos.x + 1.f, film_pos.y));
                    rays.auxilary_ray_dy_offset = ctx.camera->generate_ray(Vector2(film_pos.x, film_pos.y + 1.f));

                    Shading_Context shading_ctx(ctx, thread_ctx, rays, isect);

                    // debug visualization of samples with adjusted shading normal.
                    /*if (shading_ctx.shading_normal_adjusted) {
                        tile.add_sample(film_pos, Color_Red); 
                        continue;
                    }*/

                    ColorRGB radiance = estimate_direct_lighting(ctx, thread_ctx, shading_ctx);
                    tile.add_sample(film_pos, radiance);
                }
                else if (ctx.has_environment_light_sampler) {
                    ColorRGB radiance = ctx.environment_light_sampler.get_radiance_for_direction(ray.direction);
                    tile.add_sample(film_pos, radiance);
                }

            }
        }
    }
    film.merge_tile(tile);
}

static bool thread_context_initialized[32];
static Thread_Context thread_contexts[32];

void render_reference_image(const std::string& input_file, const Renderer_Options& options) {
    // Initialize renderer
    initalize_EWA_filter_weights(256, 2.f);

    // Load project
    printf("Loading project: %s\n", input_file.c_str());
    Timestamp t_load;
    Scene scene = load_scene(input_file);
    Scene_KdTree scene_kdtree = load_scene_kdtree(scene);

    Camera camera(scene.view_points[0], Vector2(scene.image_resolution), scene.camera_fov_y, scene.z_is_up);

    Bounds2i render_region = scene.render_region;
    if (render_region == Bounds2i{})
        render_region = { {0, 0}, scene.image_resolution }; // default render region

    ASSERT(render_region.p0 >= Vector2i{});
    ASSERT(render_region.p1 <= scene.image_resolution);
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

    Scene_Context ctx;
    ctx.scene = &scene;
    ctx.camera = &camera;
    ctx.acceleration_structure = &scene_kdtree;
    ctx.lights = scene.lights;
    ctx.materials = scene.materials;
    init_pixel_sampler_config(ctx.pixel_sampler_config, ctx);

    // Load textures.
    {
        Image_Texture::Init_Params init_params;
        init_params.generate_mips = true;

        ctx.textures.reserve(scene.texture_names.size());
        for (const std::string& texture_name : scene.texture_names) {
            std::string path = (fs::path(scene.path).parent_path() / texture_name).string();
            std::string ext = get_extension(path);

            init_params.decode_srgb = ext != ".exr";

            Image_Texture texture;
            texture.initialize_from_file(path, init_params);
            ctx.textures.push_back(std::move(texture));
        }
    }

    // Init environment map sampling.
    {
        if (scene.lights.has_environment_light) {
            const Environment_Light& light = scene.lights.environment_light;
            ASSERT(light.environment_map_index != -1);
            const Image_Texture& environment_map = ctx.textures[light.environment_map_index];

            ctx.environment_light_sampler.light = &light;
            ctx.environment_light_sampler.environment_map = &environment_map;
            ctx.environment_light_sampler.radiance_distribution.initialize_from_latitude_longitude_radiance_map(environment_map);
            ctx.has_environment_light_sampler = true;
        }
    }

    printf("Project loaded in %d ms\n", int(elapsed_milliseconds(t_load)));

    // Render image
    Timestamp t;

    if (options.thread_count == 1) {
        thread_contexts[0].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_contexts[0].pixel_sampler.init(&ctx.pixel_sampler_config);
        thread_context_initialized[0] = true;

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
                render_tile(ctx, thread_contexts[0], tile_sample_bounds, tile_pixel_bounds, rng_seed, film);
            }
        }
    } else {
        struct Render_Tile_Task : public enki::ITaskSet {
            Scene_Context* ctx;
            Bounds2i tile_sample_bounds;
            Bounds2i tile_pixel_bounds;
            uint64_t rng_seed;
            Film* film;

            void ExecuteRange(enki::TaskSetPartition, uint32_t threadnum) override {
                ASSERT(threadnum < 32);
                if (!thread_context_initialized[threadnum]) {
                    thread_contexts[threadnum].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
                    thread_contexts[threadnum].pixel_sampler.init(&ctx->pixel_sampler_config);
                    thread_context_initialized[threadnum] = true;
                }
                render_tile(*ctx, thread_contexts[threadnum], tile_sample_bounds, tile_pixel_bounds, rng_seed, *film);
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
    printf("Image rendered in %d ms\n", time);

    write_exr_image("image.exr",  image.data(), render_region.size().x, render_region.size().y);
}
