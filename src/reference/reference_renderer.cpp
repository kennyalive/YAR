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
#include "meow-hash/meow_hash_x64_aesni.h"
#include "tinyexr/tinyexr.h"

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

static std::vector<KdTree> load_geometry_kdtrees(const Scene& scene, const std::vector<Triangle_Mesh_Geometry_Data>& geometry_datas,
    std::array<int, Geometry_Type_Count>* geometry_type_offsets, bool force_rebuild_cache)
{
    fs::path kdtree_cache_directory = get_data_directory() / "kdtree-cache" / get_project_unique_name(scene.path);
    bool cache_exists = fs_exists(kdtree_cache_directory);

    // Check --force-rebuild-kdtree-cache command line option.
    if (cache_exists && force_rebuild_cache) {
        if (!fs_delete_directory(kdtree_cache_directory))
            error("Failed to delete kdtree cache (%s) when handling --force-update-kdtree-cache command", kdtree_cache_directory.c_str());
        cache_exists = false;
    }

    // Create kdtree cache if necessary.
    if (!cache_exists) {
        Timestamp t;
        printf("Kdtree cache is not found. Building kdtree cache: ");

        if (!fs_create_directories(kdtree_cache_directory))
            error("Failed to create kdtree cache directory: %s\n", kdtree_cache_directory.string().c_str());

        for (size_t i = 0; i < geometry_datas.size(); i++) {
            KdTree kdtree = build_triangle_mesh_kdtree(&geometry_datas[i]);
            fs::path kdtree_file = kdtree_cache_directory / (std::to_string(i) + ".kdtree");
            kdtree.save(kdtree_file.string());
        }
        printf("%.2fs\n", elapsed_milliseconds(t) / 1e3f);
    }

    // Load triangle mesh kdtrees.
    printf("Loading kdtree cache: ");
    std::vector<KdTree> kdtrees;
    kdtrees.reserve(scene.geometries.triangle_meshes.size());

    geometry_type_offsets->fill(0);
    (*geometry_type_offsets)[static_cast<int>(Geometry_Type::triangle_mesh)] = (int)kdtrees.size();

    for (size_t i = 0; i < geometry_datas.size(); i++) {
        fs::path kdtree_file = kdtree_cache_directory / (std::to_string(i) + ".kdtree");
        KdTree kdtree = KdTree::load(kdtree_file.string());
        kdtree.set_geometry_data(&geometry_datas[i]);
        kdtrees.push_back(std::move(kdtree));
    }
    printf("done\n");
    return kdtrees;
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
    uint64_t debug_counter = 0; // can be used in conditional breakpoint to get to problematic pixel+sample

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

                Differential_Rays differential_rays;
                differential_rays.dx_ray = scene_ctx.camera->generate_ray(Vector2(film_pos.x + 1.f, film_pos.y));
                differential_rays.dy_ray = scene_ctx.camera->generate_ray(Vector2(film_pos.x, film_pos.y + 1.f));
                // The above differential rays are generated with one pixel offset which means they estimate footprint
                // of the entire pixel. When we have many samples per pixel then we need to estimate footprint
                // that corresponds to a single sample (more precisely the area of influence of the sample).
                {
                    float scale = 1.f / std::sqrt((float)scene_ctx.pixel_sampler_config.get_samples_per_pixel());
                    differential_rays.dx_ray.direction = ray.direction + (differential_rays.dx_ray.direction - ray.direction) * scale;
                    differential_rays.dx_ray.direction.normalize();
                    differential_rays.dy_ray.direction = ray.direction + (differential_rays.dy_ray.direction - ray.direction) * scale;
                    differential_rays.dy_ray.direction.normalize();
                }

                ColorRGB radiance;
                if (scene_ctx.scene->raytracer_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::direct_lighting)
                    radiance = estimate_direct_lighting(thread_ctx, ray, differential_rays);
                else if (scene_ctx.scene->raytracer_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::path_tracer)
                    radiance = estimate_path_contribution(thread_ctx, ray, differential_rays);

                ASSERT(radiance.is_finite());
                tile.add_sample(film_pos, radiance);

                float luminance = radiance.luminance();
                luminance_sum += luminance;
                luminance_sq_sum += luminance * luminance;
                debug_counter++;
            } while (thread_ctx.pixel_sampler.next_sample_vector());

            if (thread_ctx.pixel_sampler.config->get_samples_per_pixel() > 1) {
                int n = thread_ctx.pixel_sampler.config->get_samples_per_pixel();
                double tile_variance = (luminance_sq_sum - luminance_sum * luminance_sum / n) / (n * (n - 1));
                // rounding errors might introduce negative values, strictly mathematically tile_variance can't be negative
                tile_variance = std::max(0.0, tile_variance); 
                tile_variance_accumulator += tile_variance;
            }
        }
    }

    if (thread_ctx.pixel_sampler.config->get_samples_per_pixel() > 1) {
        thread_ctx.variance_accumulator += tile_variance_accumulator;
        thread_ctx.variance_count += sample_bounds.area();
    }

    film.merge_tile(tile);
}

void render_reference_image(const std::string& input_file, const Renderer_Options& options) {
    // Initialize renderer
    initalize_EWA_filter_weights(256, 2.f);

    // Load project
    printf("Loading project: %s\n", input_file.c_str());
    Timestamp t_load;
    Scene scene = load_scene(input_file);

    std::vector<Triangle_Mesh_Geometry_Data> triangle_mesh_geometry_datas(scene.geometries.triangle_meshes.size());
    for (size_t i = 0; i < scene.geometries.triangle_meshes.size(); i++) {
        triangle_mesh_geometry_datas[i].mesh = &scene.geometries.triangle_meshes[i];
    }

    std::array<int, Geometry_Type_Count> geometry_type_offsets;
    std::vector<KdTree> geometry_kdtrees = load_geometry_kdtrees(scene, triangle_mesh_geometry_datas,
        &geometry_type_offsets, options.force_rebuild_kdtree_cache);

    Scene_Geometry_Data scene_geometry_data;
    scene_geometry_data.scene_objects = &scene.objects;
    scene_geometry_data.kdtrees = &geometry_kdtrees;
    scene_geometry_data.geometry_type_offsets = geometry_type_offsets;

    Timestamp t_scene_kdtree;
    printf("Building scene kdtree: ");
    KdTree scene_kdtree = build_scene_kdtree(&scene_geometry_data);
    printf("%.2fs\n", elapsed_milliseconds(t_scene_kdtree) / 1e3f);

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

            init_params.decode_srgb = (ext != ".exr" && ext != ".pfm");

            Image_Texture texture;
            texture.initialize_from_file(path, init_params);
            ctx.textures.push_back(std::move(texture));
        }
    }
    for (size_t i = 0; i < scene.geometries.triangle_meshes.size(); i++) {
        if (scene.geometries.triangle_meshes[i].alpha_texture_index >= 0) {
            triangle_mesh_geometry_datas[i].alpha_texture = &ctx.textures[scene.geometries.triangle_meshes[i].alpha_texture_index];
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

    std::vector<Thread_Context> thread_contexts;
    std::vector<uint8_t> is_thread_context_initialized;

    if (options.render_tile_index >= 0) {
        thread_contexts.resize(1);
        thread_contexts[0].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_contexts[0].pixel_sampler.init(&ctx.pixel_sampler_config, &thread_contexts[0].rng);
        thread_contexts[0].renderer_options = &options;
        thread_contexts[0].scene_context = &ctx;
        is_thread_context_initialized.resize(1, true);

        render_tile(ctx, thread_contexts[0], options.render_tile_index, film);
    }
    else if (options.thread_count == 1) {
        thread_contexts.resize(1);
        thread_contexts[0].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_contexts[0].pixel_sampler.init(&ctx.pixel_sampler_config, &thread_contexts[0].rng);
        thread_contexts[0].renderer_options = &options;
        thread_contexts[0].scene_context = &ctx;
        is_thread_context_initialized.resize(1, true);

        for (int tile_index = 0; tile_index < film.get_tile_count(); tile_index++) {
            render_tile(ctx, thread_contexts[0], tile_index, film);
        }
    }
    else {
        struct Render_Tile_Task : public enki::ITaskSet {
            const Renderer_Options* options;
            uint8_t* p_is_thread_context_initialized;
            Thread_Context* p_thread_contexts;
            uint32_t thread_context_count;
            Scene_Context* ctx;
            int tile_index;
            Film* film;

            void ExecuteRange(enki::TaskSetPartition, uint32_t threadnum) override {
                ASSERT(threadnum < thread_context_count);
                if (!p_is_thread_context_initialized[threadnum]) {
                    initialize_fp_state();
                    p_thread_contexts[threadnum].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
                    p_thread_contexts[threadnum].pixel_sampler.init(&ctx->pixel_sampler_config, &p_thread_contexts[threadnum].rng);
                    p_thread_contexts[threadnum].renderer_options = options;
                    p_thread_contexts[threadnum].scene_context = ctx;
                    p_is_thread_context_initialized[threadnum] = true;
                }
                render_tile(*ctx, p_thread_contexts[threadnum], tile_index, *film);
            }
        };

        enki::TaskScheduler task_scheduler;
        if (options.thread_count > 0)
            task_scheduler.Initialize(options.thread_count);
        else
            task_scheduler.Initialize();

        thread_contexts.resize(task_scheduler.GetNumTaskThreads());
        is_thread_context_initialized.resize(task_scheduler.GetNumTaskThreads(), false);

        std::vector<Render_Tile_Task> tasks(film.get_tile_count());
        for(int tile_index = 0; tile_index < film.get_tile_count(); tile_index++) {
            Render_Tile_Task task{};
            task.options  = &options;
            task.p_is_thread_context_initialized = is_thread_context_initialized.data();
            task.p_thread_contexts = thread_contexts.data();
            task.thread_context_count = (uint32_t)thread_contexts.size();
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
        for (auto [i, initialized] : enumerate(is_thread_context_initialized)) {
            if (initialized) {
                variance_accumulator += thread_contexts[i].variance_accumulator;
                variance_count += thread_contexts[i].variance_count;
            }
        }
        double variance_estimate  = variance_accumulator / variance_count;
        printf("Variance estimate %.6f, stddev %.6f\n", variance_estimate, std::sqrt(variance_estimate));
    }

    // Create output image.
    Image image = film.get_image();

    const bool adjust_render_region_to_full_res_image =
        !options.crop_image_by_render_region &&
        render_region != Bounds2i{ {0, 0}, scene.image_resolution };

    if (adjust_render_region_to_full_res_image) {
        const ColorRGB* src_pixel = image.data.data();

        std::vector<ColorRGB> full_res_image(scene.image_resolution.x * scene.image_resolution.y);
        int dst_pixel_offset = render_region.p0.y * scene.image_resolution.x +  render_region.p0.x;

        for (int y = 0; y < image.height; y++) {
            for (int x = 0; x < image.width; x++) {
                full_res_image[dst_pixel_offset + x] = *src_pixel++;
            }
            dst_pixel_offset += scene.image_resolution.x;
        }
        image.data.swap(full_res_image);
        image.width = scene.image_resolution.x;
        image.height = scene.image_resolution.y;
    }

    if (options.flip_image_horizontally) {
        ColorRGB* row = image.data.data();
        for (int y = 0; y < image.height; y++) {
            ColorRGB* here = row;
            ColorRGB* there = row + image.width - 1;
            while (here < there) {
                std::swap(*here++, *there--);
            }
            row += image.width;
        }
    }

    // Write image to disk.
    std::string output_filename = fs::path(scene.path).stem().string() + options.output_filename_suffix + ".exr";
    std::vector<EXRAttribute> custom_attributes;
    if (!image.write_exr(output_filename, custom_attributes)) {
        error("Failed to save rendered image: %s", output_filename.c_str());
    }
    printf("Saved rendered image to file: %s\n", output_filename.c_str());
}
