#include "std.h"
#include "lib/common.h"
#include "reference_renderer.h"

#include "camera.h"
#include "context.h"
#include "direct_lighting.h"
#include "film.h"
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

static void init_textures(const Scene& scene, Scene_Context& scene_ctx)
{
    // Load textures.
    Image_Texture::Init_Params init_params;
    init_params.generate_mips = true;

    scene_ctx.textures.reserve(scene.texture_names.size());
    for (const std::string& texture_name : scene.texture_names) {
        std::string path = (fs::path(scene.path).parent_path() / texture_name).string();
        std::string ext = get_extension(path);

        init_params.decode_srgb = (ext != ".exr" && ext != ".pfm");

        Image_Texture texture;
        texture.initialize_from_file(path, init_params);
        scene_ctx.textures.push_back(std::move(texture));
    }

    // Init environment map sampling.
    if (scene.lights.has_environment_light) {
        const Environment_Light& light = scene.lights.environment_light;
        ASSERT(light.environment_map_index != -1);
        const Image_Texture& environment_map = scene_ctx.textures[light.environment_map_index];

        scene_ctx.environment_light_sampler.light = &light;
        scene_ctx.environment_light_sampler.environment_map = &environment_map;
        scene_ctx.environment_light_sampler.radiance_distribution.initialize_from_latitude_longitude_radiance_map(environment_map);
        scene_ctx.has_environment_light_sampler = true;
    }
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

struct KdTree_Data {
    std::vector<Triangle_Mesh_Geometry_Data> triangle_mesh_geometry_data;
    std::vector<KdTree> geometry_kdtrees;
    Scene_Geometry_Data scene_geometry_data;
    KdTree scene_kdtree;

    void initialize(const Scene& scene, const Renderer_Options& options, const std::vector<Image_Texture>& textures)
    {
        const auto& meshes = scene.geometries.triangle_meshes;
        triangle_mesh_geometry_data.resize(meshes.size());
        for (size_t i = 0; i < meshes.size(); i++) {
            triangle_mesh_geometry_data[i].mesh = &meshes[i];

            if (meshes[i].alpha_texture_index >= 0) {
                triangle_mesh_geometry_data[i].alpha_texture = &textures[meshes[i].alpha_texture_index];
            }
        }

        std::array<int, Geometry_Type_Count> geometry_type_offsets;
        geometry_kdtrees = load_geometry_kdtrees(scene, triangle_mesh_geometry_data, &geometry_type_offsets,
            options.force_rebuild_kdtree_cache);

        scene_geometry_data.scene_objects = &scene.objects;
        scene_geometry_data.kdtrees = &geometry_kdtrees;
        scene_geometry_data.geometry_type_offsets = geometry_type_offsets;

        Timestamp t_scene_kdtree;
        printf("Building scene kdtree: ");
        scene_kdtree = build_scene_kdtree(&scene_geometry_data);
        printf("%.2fs\n", elapsed_seconds(t_scene_kdtree));
    }
};

static void init_pixel_sampler_config(Stratified_Pixel_Sampler_Configuration& pixel_sampler_config, Scene_Context& scene_ctx)
{
    const Raytracer_Config& rt_config = scene_ctx.raytracer_config;
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

    const float max_rgb_component_value = scene_ctx.raytracer_config.max_rgb_component_value_of_film_sample;
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

                Ray ray = scene_ctx.camera.generate_ray(film_pos);

                Differential_Rays differential_rays;
                differential_rays.dx_ray = scene_ctx.camera.generate_ray(Vector2(film_pos.x + 1.f, film_pos.y));
                differential_rays.dy_ray = scene_ctx.camera.generate_ray(Vector2(film_pos.x, film_pos.y + 1.f));
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
                if (scene_ctx.raytracer_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::direct_lighting)
                    radiance = estimate_direct_lighting(thread_ctx, ray, differential_rays);
                else if (scene_ctx.raytracer_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::path_tracer)
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

static Image render_scene(const Scene_Context& scene_ctx, const Renderer_Options& options, Bounds2i render_region)
{
    Film_Filter film_filter = [cfg = scene_ctx.raytracer_config]()
    {
        if (cfg.pixel_filter_type == Raytracer_Config::Pixel_Filter_Type::box)
            return get_box_filter(cfg.pixel_filter_radius);
        
        if (cfg.pixel_filter_type == Raytracer_Config::Pixel_Filter_Type::gaussian)
            return get_gaussian_filter(cfg.pixel_filter_radius, cfg.pixel_filter_alpha);

        if (cfg.pixel_filter_type == Raytracer_Config::Pixel_Filter_Type::triangle)
            return get_triangle_filter(cfg.pixel_filter_radius);

        ASSERT(!"render_image: Unknown filter type");
        return Film_Filter{};
    }();
    Film film(render_region, film_filter);

    std::vector<Thread_Context> thread_contexts;
    std::vector<uint8_t> is_thread_context_initialized;

    if (options.render_tile_index >= 0) {
        thread_contexts.resize(1);
        thread_contexts[0].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_contexts[0].pixel_sampler.init(&scene_ctx.pixel_sampler_config, &thread_contexts[0].rng);
        thread_contexts[0].renderer_options = &options;
        thread_contexts[0].scene_context = &scene_ctx;
        is_thread_context_initialized.resize(1, true);

        render_tile(scene_ctx, thread_contexts[0], options.render_tile_index, film);
    }
    else if (options.thread_count == 1) {
        thread_contexts.resize(1);
        thread_contexts[0].memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_contexts[0].pixel_sampler.init(&scene_ctx.pixel_sampler_config, &thread_contexts[0].rng);
        thread_contexts[0].renderer_options = &options;
        thread_contexts[0].scene_context = &scene_ctx;
        is_thread_context_initialized.resize(1, true);

        for (int tile_index = 0; tile_index < film.get_tile_count(); tile_index++) {
            render_tile(scene_ctx, thread_contexts[0], tile_index, film);
        }
    }
    else {
        struct Render_Tile_Task : public enki::ITaskSet {
            const Renderer_Options* options;
            uint8_t* p_is_thread_context_initialized;
            Thread_Context* p_thread_contexts;
            uint32_t thread_context_count;
            const Scene_Context* ctx;
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
        for (int tile_index = 0; tile_index < film.get_tile_count(); tile_index++) {
            Render_Tile_Task task{};
            task.options = &options;
            task.p_is_thread_context_initialized = is_thread_context_initialized.data();
            task.p_thread_contexts = thread_contexts.data();
            task.thread_context_count = (uint32_t)thread_contexts.size();
            task.ctx = &scene_ctx;
            task.tile_index = tile_index;
            task.film = &film;
            tasks[tile_index] = task;
            task_scheduler.AddTaskSetToPipe(&tasks[tile_index]);
        }
        task_scheduler.WaitforAllAndShutdown();
    }

    if (scene_ctx.pixel_sampler_config.get_samples_per_pixel() > 1) {
        double variance_accumulator = 0.0;
        int64_t variance_count = 0;
        for (auto [i, initialized] : enumerate(is_thread_context_initialized)) {
            if (initialized) {
                variance_accumulator += thread_contexts[i].variance_accumulator;
                variance_count += thread_contexts[i].variance_count;
            }
        }
        double variance_estimate = variance_accumulator / variance_count;
        printf("Variance estimate %.6f, stddev %.6f\n", variance_estimate, std::sqrt(variance_estimate));
    }

    return film.get_image();
}

struct EXR_Attributes_Writer {
    static constexpr int buffer_size = 4 * 1024;
    unsigned char value_buffer[buffer_size];
    unsigned char* buffer_ptr = value_buffer;
    std::vector<EXRAttribute> attributes;

    void add_string_attribute(const char* name, const char* value) {
        int size = (int)strlen(value); // string attributes do not require null terminator to be included
        add_attribute(name, "string", value, size);
    }
    void add_integer_attribute(const char* name, int value) {
        add_attribute(name, "int", &value, sizeof(int));
    }
    void add_float_attribute(const char* name, float value) {
        add_attribute(name, "float", &value, sizeof(float));
    }
    void add_attribute(const char* name, const char* type, const void* value, int size) {
        ASSERT(buffer_ptr + size <= value_buffer + buffer_size);
        memcpy(buffer_ptr, value, size);
        auto value_ptr = buffer_ptr;
        buffer_ptr += size;

        attributes.push_back(EXRAttribute{});
        EXRAttribute& attrib = attributes.back();
        ASSERT(strlen(name) < 256);
        strcpy(attrib.name, name);
        ASSERT(strlen(type) < 256);
        strcpy(attrib.type, type);
        attrib.value = value_ptr;
        attrib.size = size;
    }
};

struct EXR_Custom_Attributes {
    std::string input_file;
    float load_time = 0.f;
    float render_time = 0.f;
};

static void create_output_image(const Bounds2i& render_region, const Vector2i& canvas_resolution,
    const EXR_Custom_Attributes& exr_attributes, const Renderer_Options& options, Image&& image)
{
    if (!options.crop_image_by_render_region) {
        // Render region should be placed into a proper canvas position
        // when we render only a sub-region of the entire image.
        if (render_region != Bounds2i{ {0, 0}, canvas_resolution }) {
            const ColorRGB* src_pixel = image.data.data();

            std::vector<ColorRGB> canvas_pixels(canvas_resolution.area());
            ColorRGB* dst_pixel = &canvas_pixels[render_region.p0.y * canvas_resolution.x + render_region.p0.x];

            for (int y = 0; y < image.height; y++) {
                memcpy(dst_pixel, src_pixel, image.width * sizeof(ColorRGB));
                src_pixel += image.width;
                dst_pixel += canvas_resolution.x;
            }
            image.data.swap(canvas_pixels);
            image.width = canvas_resolution.x;
            image.height = canvas_resolution.y;
        }
    }

    if (options.flip_image_horizontally)
        image.flip_horizontally();

    // Initialize EXR custom attributes.
    EXR_Attributes_Writer attrib_writer;
    attrib_writer.add_string_attribute("yar_name", "YAR");
    attrib_writer.add_string_attribute("yar_render_device", "cpu");
    attrib_writer.add_string_attribute("yar_input_file", exr_attributes.input_file.c_str());
    attrib_writer.add_float_attribute("yar_load_time", exr_attributes.load_time);
    attrib_writer.add_float_attribute("yar_render_time", exr_attributes.render_time);

    // Write file to disk.
    std::string output_filename = fs::path(exr_attributes.input_file).stem().string() + options.output_filename_suffix + ".exr";
    if (!image.write_exr(output_filename, attrib_writer.attributes)) {
        error("Failed to save rendered image: %s", output_filename.c_str());
    }
    printf("Saved rendered image to file: %s\n", output_filename.c_str());
}

void cpu_renderer_render(const std::string& input_file, const Renderer_Options& options)
{
    Timestamp t_start;

    //
    // Load project file.
    //
    printf("Loading project: %s\n", input_file.c_str());
    Scene scene = load_scene(input_file);

    //
    // Initialize scene.
    //
    Scene_Context scene_ctx;
    scene_ctx.raytracer_config = scene.raytracer_config;

    scene_ctx.camera = Camera(scene.view_points[0],
        Vector2(scene.image_resolution),
        scene.camera_fov_y,
        scene.z_is_up);

    // Textures should be initialized before kdtrees,
    // kdtrees might store texture references for transparency testing
    init_textures(scene, scene_ctx);

    KdTree_Data kdtree_data;
    kdtree_data.initialize(scene, options, scene_ctx.textures);
    scene_ctx.acceleration_structure = &kdtree_data.scene_kdtree;

    scene_ctx.materials = scene.materials;
    scene_ctx.lights = scene.lights;

    init_pixel_sampler_config(scene_ctx.pixel_sampler_config, scene_ctx);

    float load_time = elapsed_seconds(t_start);
    printf("Project loaded in %.3f seconds\n", load_time);

    //
    // Render scene.
    //
    const Bounds2i render_region = [&options, &scene]() {
        Bounds2i rr;
        if (options.render_region != Bounds2i{})
            rr = options.render_region;
        else if (scene.render_region != Bounds2i{})
            rr = scene.render_region;
        else
            rr = { {0, 0}, scene.image_resolution };

        ASSERT(rr.p0 >= Vector2i{});
        ASSERT(rr.p0 < rr.p1);
        ASSERT(rr.p1 <= scene.image_resolution);
        return rr;
    }();

    Timestamp t_render;
    Image image = render_scene(scene_ctx, options, render_region);
    float render_time = elapsed_seconds(t_render);
    printf("Image rendered in %.3f seconds\n", render_time);

    //
    // Save output image.
    //
    EXR_Custom_Attributes exr_attributes{
        .input_file = input_file,
        .load_time = load_time,
        .render_time = render_time
    };
    create_output_image(render_region, scene.image_resolution, exr_attributes, options, std::move(image));
}
