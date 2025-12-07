#include "std.h"
#include "lib/common.h"
#include "reference_renderer.h"

#include "camera.h"
#include "direct_lighting.h"
#include "film.h"
#include "path_tracing.h"
#include "scene_context.h"
#include "shading_context.h"
#include "thread_context.h"

#include "lib/math.h"
#include "lib/random.h"
#include "lib/scene_loader.h"
#include "lib/vector.h"

#include "tinyexr/tinyexr.h"

constexpr int time_category_field_width = 21; // for printf 'width' specifier

static void init_textures(const Scene& scene, Scene_Context& scene_ctx)
{
    // Load textures.
    if (!scene.texture_descriptors.empty()) {
        std::atomic_int texture_counter{ 0 };
        scene_ctx.textures.resize(scene.texture_descriptors.size());

        auto load_texture_thread_func = [
            &scene,
            &scene_ctx,
            &texture_counter
        ]
        {
            initialize_fp_state();
            int index = texture_counter.fetch_add(1);
            while (index < scene.texture_descriptors.size()) {
                const Texture_Descriptor& texture_desc = scene.texture_descriptors[index];
                Image_Texture texture;

                if (!texture_desc.file_name.empty()) {
                    std::string path = scene.get_resource_absolute_path(texture_desc.file_name);
                    Image_Texture::Init_Params init_params;
                    init_params.generate_mips = true;
                    init_params.decode_srgb = texture_desc.decode_srgb;
                    init_params.scale = texture_desc.scale;
                    texture.initialize_from_file(path, init_params);
                }
                else if (texture_desc.is_constant_texture) {
                    texture.initialize_from_constant_value(texture_desc.constant_value);
                }
                else {
                    ASSERT(false);
                }
                scene_ctx.textures[index] = std::move(texture);
                index = texture_counter.fetch_add(1);
            }
        };

        // Start loading threads.
        {
            int thread_count = std::max(1, (int)std::thread::hardware_concurrency());
            thread_count = std::min(thread_count, (int)scene.texture_descriptors.size());

            std::vector<std::jthread> threads;
            threads.reserve(thread_count - 1);

            for (int i = 0; i < thread_count - 1; i++) {
                threads.push_back(std::jthread(load_texture_thread_func));
            }
            load_texture_thread_func();
        }
    }

    // Init environment map sampling.
    if (scene.lights.has_environment_light) {
        const Environment_Light& light = scene.lights.environment_light;
        ASSERT(light.environment_map_index != -1);
        const Image_Texture& environment_map = scene_ctx.textures[light.environment_map_index];

        scene_ctx.environment_light_sampler.light = &light;
        scene_ctx.environment_light_sampler.environment_map = &environment_map;
        scene_ctx.environment_light_sampler.radiance_distribution.initialize_from_latitude_longitude_radiance_map(environment_map);
    }
}

static void init_pixel_sampler_config(Stratified_Pixel_Sampler_Configuration& pixel_sampler_config, Scene_Context& scene_ctx)
{
    const Raytracer_Config& rt_config = scene_ctx.raytracer_config;
    int sample_1d_count = 0;
    int sample_2d_count = 0;
    if (rt_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::path_tracer) {
        ASSERT(rt_config.max_light_bounces >= 0);
        const int sample_1d_count_per_bounce = 4; // scattering type + light index selection + scattering type for new direction + path termination probability
        const int sample_2d_count_per_bounce = 3; // light sample + bsdf sample + bsdf sample for new direction
        sample_1d_count = std::min(10, rt_config.max_light_bounces) * sample_1d_count_per_bounce;
        sample_2d_count = std::min(10, rt_config.max_light_bounces) * sample_2d_count_per_bounce;
    }
    pixel_sampler_config.init(rt_config.x_pixel_sample_count, rt_config.y_pixel_sample_count, sample_1d_count, sample_2d_count);

    if (rt_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::direct_lighting) {
        scene_ctx.array2d_registry.rectangular_light_arrays.reserve(scene_ctx.lights.diffuse_rectangular_lights.size());
        for (const Diffuse_Rectangular_Light& light : scene_ctx.lights.diffuse_rectangular_lights) {
            int k = (int)std::ceil(std::sqrt(light.sample_count));
            ASSERT(k * k >= light.sample_count);
            ASSERT((k - 1) * (k - 1) < light.sample_count);

            MIS_Array_Info info;
            info.light_array_id = pixel_sampler_config.register_array2d_samples(k, k);
            info.bsdf_wi_array_id = pixel_sampler_config.register_array2d_samples(k, k);
            info.bsdf_scattering_array_id = pixel_sampler_config.register_array1d_samples(k * k);
            info.array_size = k * k;
            scene_ctx.array2d_registry.rectangular_light_arrays.push_back(info);
        }
        scene_ctx.array2d_registry.sphere_light_arrays.reserve(scene_ctx.lights.diffuse_sphere_lights.size());
        for (const Diffuse_Sphere_Light& light : scene_ctx.lights.diffuse_sphere_lights) {
            int k = (int)std::ceil(std::sqrt(light.sample_count));
            ASSERT(k * k >= light.sample_count);
            ASSERT((k - 1) * (k - 1) < light.sample_count);

            MIS_Array_Info info;
            info.light_array_id = pixel_sampler_config.register_array2d_samples(k, k);
            info.bsdf_wi_array_id = pixel_sampler_config.register_array2d_samples(k, k);
            info.bsdf_scattering_array_id = pixel_sampler_config.register_array1d_samples(k * k);
            info.array_size = k * k;
            scene_ctx.array2d_registry.sphere_light_arrays.push_back(info);
        }
    }
}

static void init_triangle_mesh_light_samplers(const Scene& scene, Scene_Context& scene_ctx)
{
    if (scene.lights.diffuse_triangle_mesh_lights.empty()) {
        return;
    }
    std::vector<float> areas;
    scene_ctx.triangle_mesh_light_samplers.reserve(scene.lights.diffuse_triangle_mesh_lights.size());
    for (const auto& light : scene.lights.diffuse_triangle_mesh_lights) {
        Diffuse_Triangle_Mesh_Light_Sampler& sampler = scene_ctx.triangle_mesh_light_samplers.emplace_back();
        sampler.light = &light;

        const Triangle_Mesh& mesh = scene.geometries.triangle_meshes[light.triangle_mesh_index];
        sampler.mesh = &mesh;
        sampler.mesh_area = mesh.get_area();

        const int triangle_count = mesh.get_triangle_count();
        areas.resize(triangle_count);
        for (int i = 0; i < triangle_count; i++) {
            areas[i] = mesh.get_triangle_area(i);
        }
        sampler.triangle_distribution.initialize(areas.data(), (int)areas.size());
    }
}

static std::string format_tile_index(int tile_index)
{
    std::string s = std::to_string(tile_index);
    s.insert(0, s.length() <= 4 ? 4 - s.length() : 0, '0');
    return s;
}

namespace {
struct Checkpoint_Info {
    std::string input_filename;
    int total_tile_count = 0;
    int samples_per_pixel = 0;
};

struct Checkpoint_Tile_Data {
    Film_Tile tile;
    double tile_variance_accumulator = 0.0;
};

struct Checkpoint {
    std::map<int, Checkpoint_Tile_Data> finished_tiles; // tile_index -> tile
    float previous_sessions_time = 0.f;
};
}

Checkpoint start_or_resume_checkpoint(const std::string& checkpoint_directory, const Checkpoint_Info& info)
{
    const char* func_name = "start_or_resume_from_checkpoint_directory";
    fs::path metadata_file_path = fs::path(checkpoint_directory) / "checkpoint";

    // If checkpoint directory does not exist or it is an empty directory then perform
    // initialization of the checkpoint by creating checkpoint metadata file.
    if (!fs_exists(checkpoint_directory)) {
        if (!fs_create_directories(checkpoint_directory))
            error("%s: failed to create checkpoint directory: %s",
                func_name, checkpoint_directory.c_str());
    }
    if (fs_is_empty(checkpoint_directory)) {
        std::ofstream metadata_file(metadata_file_path, std::ofstream::out);
        if (!metadata_file)
            error("%s: failed to create checkpoint file: %s",
                func_name, metadata_file_path.string().c_str());

        metadata_file << "input_filename " << info.input_filename << "\n";
        metadata_file << "total_tile_count " << info.total_tile_count << "\n";
        metadata_file << "samples_per_pixer " << info.samples_per_pixel << "\n";
        // default checkpoint object describes that no tiles were finished yet
        return Checkpoint{};
    }

    // Check that we have a valid checkpoint and that metadata matches current project settings.
    if (!fs_exists(metadata_file_path))
        error("%s: %s is not a checkpoint directory: 'checkpoint' file is missing",
            func_name, checkpoint_directory.c_str());

    std::ifstream metadata_file(metadata_file_path);
    if (!metadata_file)
        error("%s: failed to open checkpoint metadata file: %s",
            func_name, metadata_file_path.string().c_str());

    auto str_to_int = [](const std::string& s) {
        int result = 0;
        auto conv_result = std::from_chars(&*s.begin(), &*s.end(), result);
        ASSERT(conv_result.ptr == &*s.end());
        return result;
    };

    std::string tag_name;
    std::string stored_input_filename;
    std::string total_tile_count_str;
    std::string samples_per_pixel_str;

    metadata_file >> tag_name; metadata_file >> stored_input_filename;
    metadata_file >> tag_name; metadata_file >> total_tile_count_str;
    metadata_file >> tag_name; metadata_file >> samples_per_pixel_str;

    if (!metadata_file)
        error("%s: failed to read all the required fields from the metadata file: %s",
            func_name, metadata_file_path.string().c_str());

    if (stored_input_filename != info.input_filename)
        error("%s: can not resume rendering because input_filename is changed.\n"
            "Checkpoint: %s, current project: %s",
            func_name, stored_input_filename.c_str(), info.input_filename.c_str());

    int stored_total_tile_count = str_to_int(total_tile_count_str);
    if (stored_total_tile_count != info.total_tile_count)
        error("%s: can not resume rendering because total_tile_count is changed.\n"
            "Checkpoint: %d, current project: %d",
            func_name, stored_total_tile_count, info.total_tile_count);

    int stored_samples_per_pixel = str_to_int(samples_per_pixel_str);
    if (stored_samples_per_pixel != info.samples_per_pixel)
        error("%s: can not resume rendering because samples_per_pixer is changed.\n"
            "Checkpoint: %d, current project: %d",
            func_name, stored_samples_per_pixel, info.samples_per_pixel);

    // Scan checkpoint directory for already finished tiles.
    Checkpoint checkpoint;
    for (const auto& entry : fs::directory_iterator(checkpoint_directory)) {
        std::string filename = entry.path().stem().string();
        if (!filename.starts_with("tile_"))
            continue;

        int tile_index = str_to_int(filename.substr(5));
        Checkpoint_Tile_Data& tile_data = checkpoint.finished_tiles[tile_index];

        std::vector<uint8_t> content = read_binary_file(entry.path().string());
        int offset = 0;

        float time;
        memcpy(&time, content.data() + offset, sizeof(float));
        offset += sizeof(float);
        checkpoint.previous_sessions_time = std::max(checkpoint.previous_sessions_time, time);

        memcpy(&tile_data.tile_variance_accumulator, content.data() + offset, sizeof(double));
        offset += sizeof(double);

        memcpy(&tile_data.tile.pixel_bounds, content.data() + offset, sizeof(Bounds2i));
        offset += sizeof(Bounds2i);

        int pixel_count = tile_data.tile.pixel_bounds.area();
        tile_data.tile.pixels.resize(pixel_count);
        memcpy(tile_data.tile.pixels.data(), content.data() + offset, pixel_count * sizeof(Film_Pixel));
    }
    return checkpoint;
}

static void write_tile_to_checkpoint_directory(const std::string& checkpoint_directory,
    const Film_Tile& tile, int tile_index, float current_render_time, double tile_variance_accumulator)
{
    const char* func_name = "write_tile_to_checkpoint_directory";

    // The first step, is to write a tile to a temporary file. If the program terminates
    // during write operation then the checpoint directory will stay in consistent state.
    fs::path temp_file_path = fs::path(checkpoint_directory) / ("temp_tile_" + format_tile_index(tile_index));
    std::ofstream temp_file(temp_file_path, std::ofstream::out | std::ofstream::binary);
    if (!temp_file)
        error("%s: failed to create file: %s", func_name, temp_file_path.string().c_str());

    // just to check we don't have padded bytes inside the structure and
    // we can serialize entire structure with a single write.
    static_assert(sizeof(Bounds2i) == 16);
    static_assert(sizeof(Film_Pixel) == 16);

    const char* data_ptr;

    data_ptr = reinterpret_cast<const char*>(&current_render_time);
    temp_file.write(data_ptr, sizeof(float));

    data_ptr = reinterpret_cast<const char*>(&tile_variance_accumulator);
    temp_file.write(data_ptr, sizeof(double));

    data_ptr = reinterpret_cast<const char*>(&tile.pixel_bounds);
    temp_file.write(data_ptr, sizeof(Bounds2i));

    data_ptr = reinterpret_cast<const char*>(tile.pixels.data());
    temp_file.write(data_ptr, tile.pixels.size() * sizeof(Film_Pixel));

    if (temp_file.fail())
        error("%s: failed to write to file: %s", func_name, temp_file_path.string().c_str());
    temp_file.close();

    // Rename temporary tile file. The assumption is that std::filesystem::rename is atomic.
    fs::path file_path = fs::path(checkpoint_directory) / ("tile_" + format_tile_index(tile_index));
    if (fs_exists(file_path))
        error("%s: tile file already exists: %s", func_name, file_path.string().c_str());
    if (!fs_rename(temp_file_path, file_path))
        error("%s: failed to rename temp file to: %s", func_name, file_path.string().c_str());
}

struct Rendering_Progress {
    std::mutex progress_update_mutex;
    int finished_tile_count = 0;
};

static Film_Tile render_tile(Thread_Context& thread_ctx, const Film& film, int tile_index,
    double* tile_variance_accumulator, Rendering_Progress* progress)
{
    const Scene_Context& scene_ctx = thread_ctx.scene_context;

    Bounds2i sample_bounds;
    Bounds2i pixel_bounds;
    film.get_tile_bounds(tile_index, sample_bounds, pixel_bounds);

    Film_Tile tile(pixel_bounds);

    ASSERT((sample_bounds.p1 <= Vector2i{0xffff + 1, 0xffff + 1}));
    ASSERT((sample_bounds.size() <= Vector2i{0xffff, 0xffff}));
    uint64_t debug_counter = 0; // can be used in conditional breakpoint to get to problematic pixel+sample

    for (int y = sample_bounds.p0.y; y < sample_bounds.p1.y; y++) {
        for (int x = sample_bounds.p0.x; x < sample_bounds.p1.x; x++) {
            uint32_t stream_id = ((uint32_t)x & 0xffffu) | ((uint32_t)y << 16);
            stream_id += (uint32_t)scene_ctx.rng_seed_offset;
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
                    radiance = trace_path(thread_ctx, ray, differential_rays);
                ASSERT(radiance.is_finite());

                float max_component_limit = scene_ctx.raytracer_config.max_rgb_component_value_of_film_sample;
                float max_component = std::max(radiance.r, std::max(radiance.g, radiance.b));
                if (max_component > max_component_limit)
                    radiance *= (max_component_limit / max_component);
                if (scene_ctx.raytracer_config.film_radiance_scale != 1.f)
                    radiance *= scene_ctx.raytracer_config.film_radiance_scale;

                tile.add_sample(film.filter, film_pos, radiance);

                float luminance = radiance.luminance();
                luminance_sum += luminance;
                luminance_sq_sum += luminance * luminance;
                debug_counter++;
            } while (thread_ctx.pixel_sampler.next_sample_vector());

            if (thread_ctx.pixel_sampler.config->get_samples_per_pixel() > 1) {
                int n = thread_ctx.pixel_sampler.config->get_samples_per_pixel();
                double pixel_variance = (luminance_sq_sum - luminance_sum * luminance_sum / n) / (n * (n - 1));
                // rounding errors might introduce negative values, strictly mathematically tile_variance can't be negative
                pixel_variance = std::max(0.0, pixel_variance);
                *tile_variance_accumulator += pixel_variance;
            }
        }
    }

    // Update rendering progress.
    {
        std::lock_guard<std::mutex> lock(progress->progress_update_mutex);

        const int all_tile_count = film.get_tile_count();
        const int finished_tile_count = ++progress->finished_tile_count;

        int previous_percentage = 100 * (finished_tile_count - 1) / all_tile_count;
        int current_percentage = 100 * finished_tile_count / all_tile_count;

        if (current_percentage > previous_percentage)
            printf("\rRendering progress: %d%%", current_percentage);
        if (finished_tile_count == all_tile_count)
            printf("\n");
    }
    return tile;
}

static Film_Filter create_film_filter(const Raytracer_Config& cfg)
{
    if (cfg.pixel_filter_type == Raytracer_Config::Pixel_Filter_Type::box)
        return get_box_filter(cfg.pixel_filter_radius);

    if (cfg.pixel_filter_type == Raytracer_Config::Pixel_Filter_Type::gaussian)
        return get_gaussian_filter(cfg.pixel_filter_radius, cfg.pixel_filter_alpha);

    if (cfg.pixel_filter_type == Raytracer_Config::Pixel_Filter_Type::triangle)
        return get_triangle_filter(cfg.pixel_filter_radius);

    ASSERT(!"create_film_filter: Unknown filter type");
    return Film_Filter{};
}

static std::vector<int> load_checkpoint(const std::string& checkpoint_directory, const Checkpoint_Info& info,
    std::vector<Film_Tile>* tiles, std::vector<double>* tile_variance_accumulators, float* previous_sessions_time)
{
    Checkpoint checkpoint = start_or_resume_checkpoint(checkpoint_directory, info);

    std::vector<int> tiles_to_render;
    tiles_to_render.reserve(info.total_tile_count - checkpoint.finished_tiles.size());
    for (int tile_index = 0; tile_index < info.total_tile_count; tile_index++) {
        auto it = checkpoint.finished_tiles.find(tile_index);
        if (it == checkpoint.finished_tiles.end()) {
            tiles_to_render.push_back(tile_index);
        }
        else {
            (*tiles)[tile_index] = std::move(it->second.tile);
            (*tile_variance_accumulators)[tile_index] = it->second.tile_variance_accumulator;
        }
    }
    *previous_sessions_time = checkpoint.previous_sessions_time;

    if (!checkpoint.finished_tiles.empty()) {
        int checkpoint_progress_percentage = 100 * (int)checkpoint.finished_tiles.size() / info.total_tile_count;
        printf("Resuming rendering from checkpoint %s\n", checkpoint_directory.c_str());
        printf("Time spent in previous sessions: %.3f seconds\n", checkpoint.previous_sessions_time);
        printf("Rendering progress: %d%%", checkpoint_progress_percentage);
        if (checkpoint_progress_percentage == 100)
            printf("\n");
    }
    else {
        printf("Created new checkpoint %s\n", checkpoint_directory.c_str());
    }
    return tiles_to_render;
}

Image render_scene(const Scene_Context& scene_ctx, double* variance_estimate, float* render_time)
{
    Timestamp render_start_timestamp;

    Film film(scene_ctx.render_region, create_film_filter(scene_ctx.raytracer_config));

    std::vector<Film_Tile> tiles(film.get_tile_count());
    std::vector<double> tile_variance_accumulators(film.get_tile_count(), 0.0);
    float previous_sessions_time = 0.f;

    std::vector<int> tiles_to_render;
    if (!scene_ctx.checkpoint_directory.empty()) {
        Checkpoint_Info info;
        info.input_filename = scene_ctx.input_filename;
        info.total_tile_count = film.get_tile_count();
        info.samples_per_pixel = scene_ctx.pixel_sampler_config.get_samples_per_pixel();

        tiles_to_render = load_checkpoint(scene_ctx.checkpoint_directory, info,
            &tiles, &tile_variance_accumulators, &previous_sessions_time);
    }
    else {
        tiles_to_render.resize(film.get_tile_count());
        for (int i = 0; i < film.get_tile_count(); i++)
            tiles_to_render[i] = i;
    }

    Rendering_Progress progress;
    progress.finished_tile_count = film.get_tile_count() - (int)tiles_to_render.size();

    std::atomic_int tile_counter{0};

    // Each rendering thread runs this function.
    // The function runs the loop where it grabs index of the next tile and renders it.
    auto render_tile_thread_func = [
            &scene_ctx,
            &tile_counter,
            &tiles_to_render,
            &tiles,
            &tile_variance_accumulators,
            &film,
            &progress,
            previous_sessions_time,
            &render_start_timestamp
    ] {
        initialize_fp_state();

        Thread_Context thread_ctx(scene_ctx);
        thread_ctx.memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_ctx.pixel_sampler.init(&scene_ctx.pixel_sampler_config, &thread_ctx.rng);

        int index = tile_counter.fetch_add(1);

        while (index < tiles_to_render.size()) {
            int tile_index = tiles_to_render[index];
            Film_Tile& tile = tiles[tile_index];
            double& tile_variance_accumulator = tile_variance_accumulators[tile_index];

            tile = render_tile(thread_ctx, film, tile_index, &tile_variance_accumulator, &progress);

            if (!scene_ctx.checkpoint_directory.empty()) {
                float current_render_time = previous_sessions_time + elapsed_seconds(render_start_timestamp);
                write_tile_to_checkpoint_directory(scene_ctx.checkpoint_directory, tile, tile_index,
                    current_render_time, tile_variance_accumulator);
            }
            index = tile_counter.fetch_add(1);
        }
        thread_ctx.memory_pool.deallocate_pool_memory();
    };

    //
    // Render tiles. The main (this) thread also runs rendering job.
    //
    const int thread_count = std::min(scene_ctx.thread_count, (int)tiles_to_render.size());
    if (thread_count > 0) {
        std::vector<std::jthread> threads;
        threads.reserve(thread_count - 1);

        for (int i = 0; i < thread_count - 1; i++)
            threads.push_back(std::jthread(render_tile_thread_func));

        render_tile_thread_func();
    }

    //
    // Merge tiles to create final image.
    //
    for (const Film_Tile& tile : tiles)
        film.merge_tile(tile);

    Image image = film.get_image();

    if (scene_ctx.pixel_sampler_config.get_samples_per_pixel() > 1) {
        double variance_accumulator = 0.0;
        int64_t variance_count = 0;
        for (int i = 0; i < film.get_tile_count(); i++) {
            variance_accumulator += tile_variance_accumulators[i];

            Bounds2i sample_bounds, temp_pixel_bounds;
            film.get_tile_bounds(i, sample_bounds, temp_pixel_bounds);
            variance_count += sample_bounds.area();
        }
        *variance_estimate = variance_accumulator / variance_count;
    }
    *render_time = previous_sessions_time + elapsed_seconds(render_start_timestamp);
    return image;
}

void init_scene_context(const Scene& scene, const Renderer_Configuration& config, Scene_Context& scene_ctx)
{
    scene_ctx.input_filename = scene.path;
    scene_ctx.checkpoint_directory = config.checkpoint_directory;
    scene_ctx.thread_count = config.thread_count;
    scene_ctx.render_region = scene.render_region;
    scene_ctx.raytracer_config = scene.raytracer_config;
    scene_ctx.camera = Camera(scene.view_points[0], Vector2(scene.film_resolution), scene.camera_fov_y, scene.z_is_up);

    // Textures should be initialized before kdtrees,
    // kdtrees might store texture references for transparency testing
    Timestamp t_textures;
    init_textures(scene, scene_ctx);
    printf("%-*s %.3f seconds\n", time_category_field_width, "Initialize textures", elapsed_seconds(t_textures));

    scene_ctx.kdtree_data.initialize(scene, scene_ctx.textures, config.rebuild_kdtree_cache);
    scene_ctx.materials = scene.materials;
    scene_ctx.material_parameters = scene.material_parameters;
    scene_ctx.lights = scene.lights;

    init_pixel_sampler_config(scene_ctx.pixel_sampler_config, scene_ctx);
    init_triangle_mesh_light_samplers(scene, scene_ctx);

    if (scene.type == Scene_Type::pbrt) {
        // TODO: we might need to distinguish between pbrt3/4.
        scene_ctx.pbrt3_scene = true;
    }
    scene_ctx.pbrt_compatibility = config.pbrt_compatibility;
    scene_ctx.rng_seed_offset = config.rng_seed_offset;
}

struct EXR_Attributes_Writer {
    static constexpr int buffer_size = 4 * 1024;
    unsigned char value_buffer[buffer_size];
    unsigned char* buffer_ptr = value_buffer;
    std::vector<EXRAttribute> attributes;
    FILE* dump_file = nullptr;

    void add_string_attribute(const char* name, const char* value, bool add_to_image = true) {
        if (add_to_image) {
            add_attribute(name, "string", value, (int)strlen(value));
        }
        if (dump_file) {
            fprintf(dump_file, "%s %s\n", name, value);
        }
    }
    void add_integer_attribute(const char* name, int value, bool add_to_image = true) {
        if (add_to_image) {
            add_attribute(name, "int", &value, sizeof(int));
        }
        if (dump_file) {
            fprintf(dump_file, "%s %d\n", name, value);
        }
    }
    void add_float_attribute(const char* name, float value, bool add_to_image = true) {
        if (add_to_image) {
            add_attribute(name, "float", &value, sizeof(float));
        }
        if (dump_file) {
            fprintf(dump_file, "%s %f\n", name, value);
        }
    }

private:
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

bool write_openexr_image(const std::string& filename, const Image& image, const EXR_Write_Params& write_params)
{
    EXR_Attributes_Writer attrib_writer;
    if (write_params.dump_attributes) {
        std::string dumpfile = fs::path(filename).replace_extension(".txt").string();
        attrib_writer.dump_file = fopen(dumpfile.c_str(), "w");
    }

    const EXR_Attributes& attribs = write_params.attributes;

    // Add attributes.
    attrib_writer.add_string_attribute("yar_build_version", "0.0");
    attrib_writer.add_integer_attribute("yar_build_asserts", ENABLE_ASSERT);
    attrib_writer.add_string_attribute("yar_render_device", "cpu");
    attrib_writer.add_string_attribute("yar_input_file", attribs.input_file.c_str());
    attrib_writer.add_integer_attribute("yar_spp", attribs.spp);

    // The CPU renderer is deterministic, so the variance does not change between renderings
    // if other parameters are the same. Don't consider variance as varying attribute.
    attrib_writer.add_float_attribute("yar_variance", attribs.variance);

    // Attributes that can vary between renderings of the same scene.
    attrib_writer.add_float_attribute("yar_load_time", attribs.load_time, write_params.enable_varying_attributes);
    attrib_writer.add_float_attribute("yar_render_time", attribs.render_time, write_params.enable_varying_attributes);

    if (attrib_writer.dump_file) {
        fclose(attrib_writer.dump_file);
        attrib_writer.dump_file = nullptr;
    }

    // Write file to disk.
    return image.write_exr(filename, write_params.enable_compression, attrib_writer.attributes);
}
