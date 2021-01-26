#include "std.h"
#include "lib/common.h"
#include "reference_renderer.h"

#include "camera.h"
#include "context.h"
#include "direct_lighting.h"
#include "film.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "path_tracing.h"
#include "shading_context.h"

#include "lib/math.h"
#include "lib/random.h"
#include "lib/scene_loader.h"
#include "lib/vector.h"

#include "enkiTS/TaskScheduler.h"
#include "half/half.h"
#include "meow-hash/meow_hash_x64_aesni.h"

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
    const Raytracer_Config& rt_config = scene_ctx.scene->raytracer_config;
    int sample_1d_count = 0;
    int sample_2d_count = 0;
    if (rt_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::path_tracer) {
        ASSERT(rt_config.max_path_length >= 1);
        const int sample_1d_count_per_bounce = 2; // light index selection + path termination probability
        const int sample_2d_count_per_bounce = 3; // MIS light sample + MIS bsdf sample + bsdf sample for new direction
        sample_1d_count = std::min(10, (rt_config.max_path_length - 1)) * sample_1d_count_per_bounce;
        sample_2d_count = std::min(10, (rt_config.max_path_length - 1)) * sample_2d_count_per_bounce;
    }
    pixel_sampler_config.init(rt_config.x_pixel_sample_count, rt_config.y_pixel_sample_count, sample_1d_count, sample_2d_count);

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

static void render_tile(const Scene_Context& ctx, Thread_Context& thread_ctx, int tile_index, Film& film) {
    thread_ctx.rng.init(0, (uint64_t)tile_index);

    Bounds2i sample_bounds;
    Bounds2i pixel_bounds;
    film.get_tile_bounds(tile_index, sample_bounds, pixel_bounds);

    Film_Tile tile(pixel_bounds, film.filter);

    double tile_variance_accumulator = 0.0;

    for (int y = sample_bounds.p0.y; y < sample_bounds.p1.y; y++) {
        for (int x = sample_bounds.p0.x; x < sample_bounds.p1.x; x++) {
            thread_ctx.pixel_sampler.next_pixel();

            // variance estimation
            double luminance_sum = 0.0;
            double luminance_sq_sum = 0.0;

            do {
                thread_ctx.memory_pool.reset();

                Vector2 film_pos = Vector2((float)x, (float)y) + thread_ctx.pixel_sampler.get_image_plane_sample();
                Ray ray = ctx.camera->generate_ray(film_pos);

                ColorRGB radiance;
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

                    if (ctx.scene->raytracer_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::direct_lighting)
                        radiance = estimate_direct_lighting(ctx, thread_ctx, shading_ctx);
                    else if (ctx.scene->raytracer_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::path_tracer)
                        radiance = estimate_path_contribution(ctx, thread_ctx, shading_ctx);
                }
                else if (ctx.has_environment_light_sampler) {
                    radiance = ctx.environment_light_sampler.get_radiance_for_direction(ray.direction);
                }
                ASSERT(radiance.is_finite());
                tile.add_sample(film_pos, radiance);

                float luminance = radiance.luminance();
                luminance_sum += luminance;
                luminance_sq_sum += luminance * luminance;
            } while (thread_ctx.pixel_sampler.next_sample_vector());

            if (thread_ctx.pixel_sampler.config->get_samples_per_pixel() > 1) {
                int n = thread_ctx.pixel_sampler.config->get_samples_per_pixel();
                tile_variance_accumulator += (luminance_sq_sum - luminance_sum * luminance_sum / n) / (n * (n - 1));
            }
        }
    }

    if (thread_ctx.pixel_sampler.config->get_samples_per_pixel() > 1) {
        thread_ctx.variance_accumulator += tile_variance_accumulator;
        thread_ctx.variance_count += sample_bounds.area();
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

    Bounds2i render_region;
    if (options.render_region != Bounds2i{})
        render_region = options.render_region;
    else if (scene.render_region != Bounds2i{})
        render_region = scene.render_region;
    else
        render_region = { {0, 0}, scene.image_resolution };

    ASSERT(render_region.p0 >= Vector2i{});
    ASSERT(render_region.p1 <= scene.image_resolution);
    ASSERT(render_region.p0 < render_region.p1);

    const float filter_radius = scene.raytracer_config.pixel_filter_radius;
    Film_Filter film_filter;
    if (scene.raytracer_config.pixel_filter_type == Raytracer_Config::Pixel_Filter_Type::box) {
        film_filter = get_box_filter(filter_radius);
    }
    else if (scene.raytracer_config.pixel_filter_type == Raytracer_Config::Pixel_Filter_Type::gaussian) {
        film_filter = get_gaussian_filter(filter_radius, scene.raytracer_config.pixel_filter_alpha);
    }
    else if (scene.raytracer_config.pixel_filter_type == Raytracer_Config::Pixel_Filter_Type::triangle) {
        film_filter = get_triangle_filter(filter_radius);
    }
    else {
        error("Unknown filter type");
    }

    Film film(render_region, film_filter);

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

    if (options.render_tile_index >= 0) {
        thread_contexts[0].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_contexts[0].pixel_sampler.init(&ctx.pixel_sampler_config, &thread_contexts[0].rng);
        thread_context_initialized[0] = true;

        render_tile(ctx, thread_contexts[0], options.render_tile_index, film);
    }
    else if (options.thread_count == 1) {
        thread_contexts[0].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_contexts[0].pixel_sampler.init(&ctx.pixel_sampler_config, &thread_contexts[0].rng);
        thread_context_initialized[0] = true;

        for (int tile_index = 0; tile_index < film.get_tile_count(); tile_index++) {
            render_tile(ctx, thread_contexts[0], tile_index, film);
        }
    }
    else {
        struct Render_Tile_Task : public enki::ITaskSet {
            Scene_Context* ctx;
            int tile_index;
            Film* film;

            void ExecuteRange(enki::TaskSetPartition, uint32_t threadnum) override {
                ASSERT(threadnum < 32);
                if (!thread_context_initialized[threadnum]) {
                    initialize_fp_state();
                    thread_contexts[threadnum].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
                    thread_contexts[threadnum].pixel_sampler.init(&ctx->pixel_sampler_config, &thread_contexts[threadnum].rng);
                    thread_context_initialized[threadnum] = true;
                }
                render_tile(*ctx, thread_contexts[threadnum], tile_index, *film);
            }
        };

        enki::TaskScheduler task_scheduler;
        task_scheduler.Initialize();
        std::vector<Render_Tile_Task> tasks(film.get_tile_count());
        for(int tile_index = 0; tile_index < film.get_tile_count(); tile_index++) {
            Render_Tile_Task task{};
            task.ctx = &ctx;
            task.tile_index = tile_index;
            task.film = &film;
            tasks[tile_index] = task;
            task_scheduler.AddTaskSetToPipe(&tasks[tile_index]);
        }
        task_scheduler.WaitforAllAndShutdown();
    }

    std::vector<ColorRGB> image = film.get_image();

    double variance_accumulator = 0.0;
    int64_t variance_count = 0;
    for (auto [i, initialized] : enumerate(thread_context_initialized)) {
        if (initialized) {
            variance_accumulator += thread_contexts[i].variance_accumulator;
            variance_count += thread_contexts[i].variance_count;
        }
    }
    double variance_estimate  = variance_accumulator / variance_count;

    int time = int(elapsed_milliseconds(t));
    printf("Image rendered in %d ms\n", time);
    printf("Variance estimate %.6f, stddev %.6f\n", variance_estimate, std::sqrt(variance_estimate));
    write_exr_image("image.exr",  image.data(), render_region.size().x, render_region.size().y);
}
