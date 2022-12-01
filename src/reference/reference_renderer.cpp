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

#include "meow-hash/meow_hash_x64_aesni.h"
#include "tinyexr/tinyexr.h"

constexpr int time_category_field_width = 21; // for printf 'width' specifier

static void init_textures(const Scene& scene, Scene_Context& scene_ctx)
{
    // Load textures.
    if (!scene.texture_descriptors.empty()) {
        std::atomic_int texture_counter{ 0 };
        scene_ctx.textures.resize(scene.texture_descriptors.size());

        Image_Texture::Init_Params init_params;
        init_params.generate_mips = true;

        auto load_texture_thread_func = [
            &scene,
            &scene_ctx,
            &init_params,
            &texture_counter
        ]
        {
            initialize_fp_state();
            int index = texture_counter.fetch_add(1);
            while (index < scene.texture_descriptors.size()) {
                const Texture_Descriptor& texture_desc = scene.texture_descriptors[index];
                Image_Texture texture;

                if (!texture_desc.file_name.empty()) {
                    std::string path = (fs::path(scene.path).parent_path() / texture_desc.file_name).string();
                    std::string ext = get_extension(path);

                    init_params.decode_srgb = texture_desc.decode_srgb;
                    texture.initialize_from_file(path, init_params);

                    if (texture_desc.scale != 1.f)
                        texture.scale_all_mips(texture_desc.scale);
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
        printf("Kdtree cache was not found\n");
        printf("%-*s", time_category_field_width, "Building kdtree cache ");

        if (!fs_create_directories(kdtree_cache_directory))
            error("Failed to create kdtree cache directory: %s\n", kdtree_cache_directory.string().c_str());

        for (size_t i = 0; i < geometry_datas.size(); i++) {
            KdTree kdtree = build_triangle_mesh_kdtree(&geometry_datas[i]);
            fs::path kdtree_file = kdtree_cache_directory / (std::to_string(i) + ".kdtree");
            kdtree.save(kdtree_file.string());
        }
        printf("%.3f seconds\n", elapsed_seconds(t));
    }

    // Load triangle mesh kdtrees.
    Timestamp t_kdtree_cache;
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
    printf("%-*s %.3f seconds\n", time_category_field_width, "Load KdTree cache", elapsed_seconds(t_kdtree_cache));
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
        scene_kdtree = build_scene_kdtree(&scene_geometry_data);
        printf("%-*s %.3f seconds\n", time_category_field_width, "Build scene KdTree", elapsed_seconds(t_scene_kdtree));
    }
};

static void init_pixel_sampler_config(Stratified_Pixel_Sampler_Configuration& pixel_sampler_config, Scene_Context& scene_ctx)
{
    const Raytracer_Config& rt_config = scene_ctx.raytracer_config;
    int sample_1d_count = 0;
    int sample_2d_count = 0;
    if (rt_config.rendering_algorithm == Raytracer_Config::Rendering_Algorithm::path_tracer) {
        ASSERT(rt_config.max_light_bounces >= 0);
        const int sample_1d_count_per_bounce = 3; // scattering initialization + light index selection + path termination probability
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
    const Scene_Context& scene_ctx = *thread_ctx.scene_context;

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

static Image render_scene(const Scene_Context& scene_ctx, const Renderer_Options& options, Bounds2i render_region,
    double* variance_estimate, float* render_time)
{
    Timestamp render_start_timestamp;

    Film film(render_region, create_film_filter(scene_ctx.raytracer_config));

    std::vector<Film_Tile> tiles(film.get_tile_count());
    std::vector<double> tile_variance_accumulators(film.get_tile_count(), 0.0);
    float previous_sessions_time = 0.f;

    std::vector<int> tiles_to_render;
    if (!options.checkpoint_directory.empty()) {
        Checkpoint_Info info;
        info.input_filename = scene_ctx.input_filename;
        info.total_tile_count = film.get_tile_count();
        info.samples_per_pixel = scene_ctx.pixel_sampler_config.get_samples_per_pixel();

        tiles_to_render = load_checkpoint(options.checkpoint_directory, info,
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
            &options,
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

        Thread_Context thread_ctx;
        thread_ctx.memory_pool.allocate_pool_memory(1 * 1024 * 1024);
        thread_ctx.pixel_sampler.init(&scene_ctx.pixel_sampler_config, &thread_ctx.rng);
        thread_ctx.renderer_options = &options;
        thread_ctx.scene_context = &scene_ctx;

        int index = tile_counter.fetch_add(1);

        while (index < tiles_to_render.size()) {
            int tile_index = tiles_to_render[index];
            Film_Tile& tile = tiles[tile_index];
            double& tile_variance_accumulator = tile_variance_accumulators[tile_index];

            tile = render_tile(thread_ctx, film, tile_index, &tile_variance_accumulator, &progress);

            if (!options.checkpoint_directory.empty()) {
                float current_render_time = previous_sessions_time + elapsed_seconds(render_start_timestamp);
                write_tile_to_checkpoint_directory(options.checkpoint_directory, tile, tile_index,
                    current_render_time, tile_variance_accumulator);
            }
            index = tile_counter.fetch_add(1);
        }
        thread_ctx.memory_pool.deallocate_pool_memory();
    };

    //
    // Render tiles. The main (this) thread also runs rendering job.
    //
    int thread_count;
    if (options.thread_count > 0)
        thread_count = options.thread_count;
    else
        thread_count = std::max(1u, std::thread::hardware_concurrency());
    thread_count = std::min(thread_count, (int)tiles_to_render.size());

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
    int spp = 0; // samples per pixel

    // Here are the attributes that vary between render sessions. We store such
    // attributes in the output file only if --openexr-varying-attributes command
    // line option is specified. The reason why we do not always write them is to
    // keep output deterministic by default.
    float load_time = 0.f;
    float render_time = 0.f;

    float variance = 0.f;
};

static void save_output_image(
    const std::string& output_filename, Image image,
    const Bounds2i& render_region, const Vector2i& film_resolution,
    const Renderer_Options& options, const EXR_Custom_Attributes& exr_attributes)
{
    ASSERT(image.width == render_region.size().x);
    ASSERT(image.height == render_region.size().y);

    if (!options.crop_image_by_render_region) {
        // Render region should be placed into a proper canvas position
        // when we render only a sub-region of the entire image.
        if (render_region != Bounds2i{ {0, 0}, film_resolution }) {
            const ColorRGB* src_pixel = image.data.data();

            std::vector<ColorRGB> film_pixels(film_resolution.area()); // the pixels outside render region will be black
            ColorRGB* dst_pixel = &film_pixels[render_region.p0.y * film_resolution.x + render_region.p0.x];

            for (int y = 0; y < image.height; y++) {
                memcpy(dst_pixel, src_pixel, image.width * sizeof(ColorRGB));
                src_pixel += image.width;
                dst_pixel += film_resolution.x;
            }
            image.data.swap(film_pixels);
            image.width = film_resolution.x;
            image.height = film_resolution.y;
        }
    }

    if (options.flip_image_horizontally)
        image.flip_horizontally();

    // Initialize EXR custom attributes.
    EXR_Attributes_Writer attrib_writer;
    attrib_writer.add_string_attribute("yar_build_version", "0.0");
    attrib_writer.add_integer_attribute("yar_build_asserts", ENABLE_ASSERT);
    attrib_writer.add_string_attribute("yar_render_device", "cpu");
    attrib_writer.add_string_attribute("yar_input_file", exr_attributes.input_file.c_str());
    attrib_writer.add_integer_attribute("yar_spp", exr_attributes.spp);

    // We have deterministic CPU rendering, so variance does not change between
    // renders if other parameters are the same. That's why we don't put variance
    // under openexr_disable_varying_attributes scope.
    attrib_writer.add_float_attribute("yar_variance", exr_attributes.variance);

    if (!options.openexr_disable_varying_attributes) {
        attrib_writer.add_float_attribute("yar_load_time", exr_attributes.load_time);
        attrib_writer.add_float_attribute("yar_render_time", exr_attributes.render_time);
    }

    // Write file to disk.
    if (!image.write_exr(output_filename, options.openexr_enable_compression, attrib_writer.attributes)) {
        error("Failed to save rendered image: %s", output_filename.c_str());
    }
    printf("Saved output image to %s\n\n", output_filename.c_str());
}

void cpu_renderer_render(const std::string& input_file, const Renderer_Options& options)
{
    Timestamp t_start;
    printf("Loading: %s\n", input_file.c_str());

    //
    // Parse project file.
    //
    Timestamp t_project;
    Scene scene = load_scene(input_file);
    printf("%-*s %.3f seconds\n", time_category_field_width, "Parse project", elapsed_seconds(t_project));

    //
    // Initialize scene.
    //
    Scene_Context scene_ctx;
    scene_ctx.input_filename = scene.path;
    scene_ctx.raytracer_config = scene.raytracer_config;

    if (options.samples_per_pixel > 0) {
        int k = (int)std::ceil(std::sqrt(options.samples_per_pixel));
        scene_ctx.raytracer_config.x_pixel_sample_count = k;
        scene_ctx.raytracer_config.y_pixel_sample_count = k;
    }

    const Vector2i film_resolution = (options.film_resolution != Vector2i{}) ? options.film_resolution : scene.film_resolution;

    scene_ctx.camera = Camera(scene.view_points[0],
        Vector2(film_resolution),
        scene.camera_fov_y,
        scene.z_is_up);

    // Textures should be initialized before kdtrees,
    // kdtrees might store texture references for transparency testing
    Timestamp t_textures;
    init_textures(scene, scene_ctx);
    printf("%-*s %.3f seconds\n", time_category_field_width, "Initialize textures", elapsed_seconds(t_textures));

    KdTree_Data kdtree_data;
    kdtree_data.initialize(scene, options, scene_ctx.textures);
    scene_ctx.acceleration_structure = &kdtree_data.scene_kdtree;

    scene_ctx.materials = scene.materials;
    scene_ctx.lights = scene.lights;

    init_pixel_sampler_config(scene_ctx.pixel_sampler_config, scene_ctx);

    float load_time = elapsed_seconds(t_start);
    printf("%-*s %.3f seconds\n\n", time_category_field_width, "Total loading time", load_time);

    //
    // Render scene.
    //
    const Bounds2i render_region = [&options, &scene]() {
        Bounds2i rr;
        if (options.render_region != Bounds2i{}) {
            rr = options.render_region;
        }
        else if (options.film_resolution != Vector2i{}) {
            // Custom film resolution invalidates scene.render_region,
            // so we set render region to match custom film resolution.
            rr = Bounds2i{ {0, 0}, options.film_resolution };
        }
        else if (scene.render_region != Bounds2i{}) {
            rr = scene.render_region;
        }
        else {
            rr = { {0, 0}, scene.film_resolution };
        }
        return rr;
    }();

    // assert that render region is within the film dimensions
    ASSERT(render_region.p0 >= Vector2i{});
    ASSERT(render_region.p0 < render_region.p1);
    ASSERT(render_region.p1 <= film_resolution);

    double variance_estimate = 0.0;
    float render_time = 0.f;
    Image image = render_scene(scene_ctx, options, render_region, &variance_estimate, &render_time);

    printf("%-*s %.3f seconds\n", 12, "Render time", render_time);
    printf("%-*s %.6f\n", 12, "Variance", variance_estimate);
    printf("%-*s %.6f\n", 12, "StdDev", std::sqrt(variance_estimate));

    //
    // Save output image.
    //
    std::string output_filename;
    if (!scene.output_filename.empty())
        output_filename = fs::path(scene.output_filename).replace_extension().string();
    else
        output_filename = fs::path(input_file).stem().string();

    if (!options.output_directory.empty())
        output_filename = (fs::path(options.output_directory) / fs::path(output_filename)).string();

    output_filename += options.output_filename_suffix;
    output_filename += ".exr"; // output is always in OpenEXR format

    EXR_Custom_Attributes exr_attributes{
        .input_file = input_file,
        .spp = scene_ctx.pixel_sampler_config.get_samples_per_pixel(),
        .load_time = load_time,
        .render_time = render_time,
        .variance = (float)variance_estimate
    };

    save_output_image(output_filename, image, render_region, film_resolution, options, exr_attributes);
}
