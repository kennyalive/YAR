#include "std.h"
#include "lib/common.h"
#include "scene_context.h"

#include "kdtree_builder.h"

#include "lib/scene.h"

constexpr int time_category_field_width = 21; // for printf 'width' specifier

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

        std::atomic_int kdtree_counter{ 0 };
        auto build_kdtree_func = [
            &kdtree_cache_directory,
                &geometry_datas,
                &kdtree_counter
        ]
            {
                initialize_fp_state();
                int index = kdtree_counter.fetch_add(1);
                while (index < geometry_datas.size()) {
                    KdTree kdtree = build_triangle_mesh_kdtree(&geometry_datas[index]);
                    fs::path kdtree_file = kdtree_cache_directory / (std::to_string(index) + ".kdtree");
                    kdtree.save(kdtree_file.string());
                    index = kdtree_counter.fetch_add(1);
                }
            };
        // Start kdtree build threads.
        {
            int thread_count = std::max(1, (int)std::thread::hardware_concurrency());
            thread_count = std::min(thread_count, (int)geometry_datas.size());

            std::vector<std::jthread> threads;
            threads.reserve(thread_count - 1);

            for (int i = 0; i < thread_count - 1; i++) {
                threads.push_back(std::jthread(build_kdtree_func));
            }
            build_kdtree_func();
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

void KdTree_Data::initialize(const Scene& scene, const std::vector<Image_Texture>& textures, bool rebuild_kdtree_cache)
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
        rebuild_kdtree_cache);

    scene_geometry_data.scene_objects = &scene.objects;
    scene_geometry_data.kdtrees = &geometry_kdtrees;
    scene_geometry_data.geometry_type_offsets = geometry_type_offsets;

    Timestamp t_scene_kdtree;
    scene_kdtree = build_scene_kdtree(&scene_geometry_data);
    printf("%-*s %.3f seconds\n", time_category_field_width, "Build scene KdTree", elapsed_seconds(t_scene_kdtree));
}
