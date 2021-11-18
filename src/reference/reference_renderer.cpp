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
    std::vector<int> geometry_type_offsets(Geometry_Type_Count, 0);
    std::vector<Geometry_KdTree> kdtrees;
    kdtrees.reserve(scene.geometries.triangle_meshes.size());

    geometry_type_offsets[static_cast<int>(Geometry_Type::triangle_mesh)] = (int)kdtrees.size();
    for (int i = 0; i < (int)scene.geometries.triangle_meshes.size(); i++) {
        fs::path kdtree_file = kdtree_cache_directory / (std::to_string(i) + ".kdtree");
        Geometry_KdTree kdtree = load_geometry_kdtree(kdtree_file.string(), &scene.geometries, {Geometry_Type::triangle_mesh, i});
        kdtrees.push_back(std::move(kdtree));
    }
    printf("done\n");

    // Build top-level scene kdtree.
    Scene_KdTree scene_kdtree;
    {
        Timestamp t;
        printf("Building scene kdtree: ");
        scene_kdtree = build_scene_kdtree(&scene, std::move(geometry_type_offsets), std::move(kdtrees));
        printf("%.2fs\n", elapsed_milliseconds(t) / 1e3f);
    }
    return scene_kdtree;
}

static void init_pixel_sampler_config(Stratified_Pixel_Sampler_Configuration& pixel_sampler_config, Scene_Context& scene_ctx) {
    const Raytracer_Config& rt_config = scene_ctx.scene->raytracer_config;
    int sample_1d_count = 0;
    int sample_2d_count = 0;
    if (rt_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::path_tracer) {
        ASSERT(rt_config.max_light_bounces >= 0);
        const int sample_1d_count_per_bounce = 2; // light index selection + path termination probability
        const int sample_2d_count_per_bounce = 3; // MIS light sample + MIS bsdf sample + bsdf sample for new direction
        sample_1d_count = std::min(10, rt_config.max_light_bounces) * sample_1d_count_per_bounce;
        sample_2d_count = std::min(10, rt_config.max_light_bounces) * sample_2d_count_per_bounce;
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

static void render_tile(const Scene_Context& scene_ctx, Thread_Context& thread_ctx, int tile_index, Film& film) {
    Bounds2i sample_bounds;
    Bounds2i pixel_bounds;
    film.get_tile_bounds(tile_index, sample_bounds, pixel_bounds);

    const float max_rgb_component_value = scene_ctx.scene->raytracer_config.max_rgb_component_value_of_film_sample;
    Film_Tile tile(pixel_bounds, film.filter, max_rgb_component_value);

    double tile_variance_accumulator = 0.0;

    ASSERT((sample_bounds.p1 <= Vector2i{0xffff + 1, 0xffff + 1}));
    ASSERT((sample_bounds.size() <= Vector2i{0xffff, 0xffff}));

    for (int y = sample_bounds.p0.y; y < sample_bounds.p1.y; y++) {
        for (int x = sample_bounds.p0.x; x < sample_bounds.p1.x; x++) {
            uint32_t stream_id = ((uint32_t)x & 0xffffu) | ((uint32_t)y << 16);
            stream_id += (uint32_t)thread_ctx.renderer_options->rng_seed_offset;
            thread_ctx.rng.init(0, stream_id);
            thread_ctx.pixel_sampler.next_pixel();
            thread_ctx.shading_context = Shading_Context{};

            // variance estimation
            double luminance_sum = 0.0;
            double luminance_sq_sum = 0.0;

            do {
                thread_ctx.memory_pool.reset();
                thread_ctx.current_dielectric_material = Null_Material; // TODO: should be part of path context
                thread_ctx.path_context = Path_Context{};

                Vector2 film_pos = Vector2((float)x, (float)y) + thread_ctx.pixel_sampler.get_image_plane_sample();

                Ray ray = scene_ctx.camera->generate_ray(film_pos);

                Auxilary_Rays auxilary_rays;
                auxilary_rays.ray_dx_offset = scene_ctx.camera->generate_ray(Vector2(film_pos.x + 1.f, film_pos.y));
                auxilary_rays.ray_dy_offset = scene_ctx.camera->generate_ray(Vector2(film_pos.x, film_pos.y + 1.f));
                // The above auxilary rays are generated with one pixel offset which means they estimate footprint
                // of the entire pixel. When we have many samples per pixel then we need to estimate footprint
                // that corresponds to a single sample (more precisely the area of influence of the sample).
                {
                    float scale = 1.f / std::sqrt((float)scene_ctx.pixel_sampler_config.get_samples_per_pixel());
                    auxilary_rays.ray_dx_offset.direction = ray.direction + (auxilary_rays.ray_dx_offset.direction - ray.direction) * scale;
                    auxilary_rays.ray_dx_offset.direction.normalize();
                    auxilary_rays.ray_dy_offset.direction = ray.direction + (auxilary_rays.ray_dy_offset.direction - ray.direction) * scale;
                    auxilary_rays.ray_dy_offset.direction.normalize();
                }

                ColorRGB radiance;
                if (scene_ctx.scene->raytracer_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::direct_lighting)
                    radiance = estimate_direct_lighting(thread_ctx, ray, auxilary_rays);
                else if (scene_ctx.scene->raytracer_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::path_tracer)
                    radiance = estimate_path_contribution(thread_ctx, ray, auxilary_rays);

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

    Film film(render_region, film_filter);

    // Render image
    Timestamp t;

    if (options.render_tile_index >= 0) {
        thread_contexts[0].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_contexts[0].pixel_sampler.init(&ctx.pixel_sampler_config, &thread_contexts[0].rng);
        thread_contexts[0].renderer_options = &options;
        thread_contexts[0].scene_context = &ctx;
        thread_context_initialized[0] = true;

        render_tile(ctx, thread_contexts[0], options.render_tile_index, film);
    }
    else if (options.thread_count == 1) {
        thread_contexts[0].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_contexts[0].pixel_sampler.init(&ctx.pixel_sampler_config, &thread_contexts[0].rng);
        thread_contexts[0].renderer_options = &options;
        thread_contexts[0].scene_context = &ctx;
        thread_context_initialized[0] = true;

        for (int tile_index = 0; tile_index < film.get_tile_count(); tile_index++) {
            render_tile(ctx, thread_contexts[0], tile_index, film);
        }
    }
    else {
        struct Render_Tile_Task : public enki::ITaskSet {
            const Renderer_Options* options;
            Scene_Context* ctx;
            int tile_index;
            Film* film;

            void ExecuteRange(enki::TaskSetPartition, uint32_t threadnum) override {
                ASSERT(threadnum < 32);
                if (!thread_context_initialized[threadnum]) {
                    initialize_fp_state();
                    thread_contexts[threadnum].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
                    thread_contexts[threadnum].pixel_sampler.init(&ctx->pixel_sampler_config, &thread_contexts[threadnum].rng);
                    thread_contexts[threadnum].renderer_options = options;
                    thread_contexts[threadnum].scene_context = ctx;
                    thread_context_initialized[threadnum] = true;
                }
                render_tile(*ctx, thread_contexts[threadnum], tile_index, *film);
            }
        };

        enki::TaskScheduler task_scheduler;
        if (options.thread_count > 0)
            task_scheduler.Initialize(options.thread_count);
        else
            task_scheduler.Initialize();

        std::vector<Render_Tile_Task> tasks(film.get_tile_count());
        for(int tile_index = 0; tile_index < film.get_tile_count(); tile_index++) {
            Render_Tile_Task task{};
            task.options  = &options;
            task.ctx = &ctx;
            task.tile_index = tile_index;
            task.film = &film;
            tasks[tile_index] = task;
            task_scheduler.AddTaskSetToPipe(&tasks[tile_index]);
        }
        task_scheduler.WaitforAllAndShutdown();
    }

    int time = int(elapsed_milliseconds(t));
    printf("Image rendered in %d ms\n", time);

    if (ctx.pixel_sampler_config.get_samples_per_pixel() > 1) {
        double variance_accumulator = 0.0;
        int64_t variance_count = 0;
        for (auto [i, initialized] : enumerate(thread_context_initialized)) {
            if (initialized) {
                variance_accumulator += thread_contexts[i].variance_accumulator;
                variance_count += thread_contexts[i].variance_count;
            }
        }
        double variance_estimate  = variance_accumulator / variance_count;
        printf("Variance estimate %.6f, stddev %.6f\n", variance_estimate, std::sqrt(variance_estimate));
    }

    // Create output image.
    std::vector<ColorRGB> image = film.get_image();
    Vector2i image_size = render_region.size();

    bool adjust_render_region_to_full_res_image = (render_region != Bounds2i{{0, 0}, scene.image_resolution}) && !options.crop_image_by_render_region;

    if (adjust_render_region_to_full_res_image) {
        const ColorRGB* src_pixel = image.data();
        int src_width = render_region.size().x;
        int src_height = render_region.size().y;

        std::vector<ColorRGB> full_res_image(scene.image_resolution.x * scene.image_resolution.y);
        int dst_pixel_offset = render_region.p0.y * scene.image_resolution.x +  render_region.p0.x;

        for (int y = 0; y < src_height; y++) {
            for (int x = 0; x < src_width; x++) {
                full_res_image[dst_pixel_offset + x] = *src_pixel++;
            }
            dst_pixel_offset += scene.image_resolution.x;
        }

        image.swap(full_res_image);
        image_size = scene.image_resolution;
    }

    if (options.flip_image_horizontally) {
        ColorRGB* row = image.data();
        for (int y = 0; y < image_size.y; y++) {
            ColorRGB* here = row;
            ColorRGB* there = row + image_size.x - 1;
            while (here < there) {
                std::swap(*here++, *there--);
            }
            row += image_size.x;
        }
    }

    write_exr_image("image.exr",  image.data(), image_size.x, image_size.y);
}
