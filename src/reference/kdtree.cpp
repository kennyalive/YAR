#include "std.h"
#include "lib/common.h"
#include "kdtree.h"

#include "image_texture.h"
#include "intersection.h"

#include "lib/scene_object.h"

constexpr int max_traversal_depth = 40;

static void intersect_triangle_mesh_geometry_data(const Ray& ray, const void* geometry_data, uint32_t primitive_index, Intersection& intersection)
{
    auto data = static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data);
    Vector3 p0, p1, p2;
    data->mesh->get_triangle(primitive_index, p0, p1, p2);

    Vector3 b;
    float t = intersect_triangle_watertight(ray, p0, p1, p2, &b);

    if (t < intersection.t) {
        // Do alpha test.
        if (data->alpha_texture != nullptr) {
            Vector2 uv = data->mesh->get_uv(primitive_index, b);
            ColorRGB alpha = data->alpha_texture->sample_bilinear(uv, 0, Wrap_Mode::repeat);
            if (alpha.r == 0.f)
                return; // skip this triangle
        }
        intersection.t = t;
        intersection.geometry_type = Geometry_Type::triangle_mesh;
        intersection.triangle_intersection.barycentrics = b;
        intersection.triangle_intersection.mesh = data->mesh;
        intersection.triangle_intersection.triangle_index = primitive_index;
    }
}

static void intersect_scene_geometry_data(const Ray& ray, const void* geometry_data, uint32_t primitive_index, Intersection& intersection)
{
    auto data = static_cast<const Scene_Geometry_Data*>(geometry_data);

    ASSERT(primitive_index >= 0 && primitive_index < data->scene_objects->size());
    const Scene_Object* scene_object = &(*data->scene_objects)[primitive_index];
    Ray ray_in_object_space = transform_ray(scene_object->world_to_object_transform, ray);

    int offset = data->geometry_type_offsets[static_cast<int>(scene_object->geometry.type)];
    int kdtree_index = offset + scene_object->geometry.index;

    if ((*data->kdtrees)[kdtree_index].intersect(ray_in_object_space, intersection)) {
        intersection.scene_object = scene_object;
    }
}

KdTree KdTree::load(const std::string& file_name)
{
    std::ifstream file(file_name, std::ios_base::in | std::ios_base::binary);
    if (!file)
        error("KdTree::load: failed to open file: %s", file_name.c_str());

    KdTree kdtree;

    // bounds
    static_assert(sizeof(Bounding_Box) == 2 * 3*sizeof(float));
    file.read(reinterpret_cast<char*>(&kdtree.bounds), sizeof(Bounding_Box));

    // geometry hash
    static_assert(sizeof(KdTree::geometry_data_hash) == sizeof(uint64_t));
    file.read(reinterpret_cast<char*>(&kdtree.geometry_data_hash), sizeof(uint64_t));

    // nodes
    uint32_t node_count = 0;
    file.read(reinterpret_cast<char*>(&node_count), 4);
    kdtree.nodes.resize(node_count);
    size_t nodes_byte_count = node_count * sizeof(KdNode);
    file.read(reinterpret_cast<char*>(kdtree.nodes.data()), nodes_byte_count);

    // primitive indices
    uint32_t index_count = 0;
    file.read(reinterpret_cast<char*>(&index_count), 4);
    kdtree.primitive_indices.resize(index_count);
    size_t indices_byte_count = index_count * 4;
    file.read(reinterpret_cast<char*>(kdtree.primitive_indices.data()), indices_byte_count);

    if (file.fail())
        error("KdTree::load: failed to read kdtree data: %s", file_name.c_str());

#if USE_KD_TILES
    if (!kdtree.nodes[0].is_leaf()) {
        std::vector<uint8_t> tiles = convert_kdtree_nodes_to_tiled_layout(kdtree);
        kdtree.tile_buffer = std::unique_ptr<uint8_t, KdTree::Tile_Buffer_Deleter>(
            (uint8_t*)_aligned_malloc(tiles.size(), cache_line_size)
            );
        memcpy(kdtree.tile_buffer.get(), tiles.data(), tiles.size());
        kdtree.tile_buffer_size = tiles.size();
    }
#endif

    return kdtree;
}

void KdTree::save(const std::string& file_name) const
{
    std::ofstream file(file_name, std::ios_base::out | std::ios_base::binary);
    if (!file)
        error("KdTree::save: failed to open file for writing: %s", file_name.c_str());

    // bounds
    file.write(reinterpret_cast<const char*>(&bounds), sizeof(Bounding_Box));

    // geometry hash
    file.write(reinterpret_cast<const char*>(&geometry_data_hash), sizeof(uint64_t));

    // nodes
    uint32_t node_count = (uint32_t)nodes.size();
    file.write(reinterpret_cast<const char*>(&node_count), sizeof(uint32_t));
    size_t nodes_byte_count = node_count * sizeof(KdNode);
    file.write(reinterpret_cast<const char*>(nodes.data()), nodes_byte_count);

    // primitive indices
    uint32_t index_count = (uint32_t)primitive_indices.size();
    file.write(reinterpret_cast<const char*>(&index_count), sizeof(uint32_t));
    size_t indices_byte_count = index_count * 4;
    file.write(reinterpret_cast<const char*>(primitive_indices.data()), indices_byte_count);

    if (file.fail())
        error("KdTree::save: failed to write kdtree data: %s", file_name.c_str());
}

bool KdTree::set_geometry_data(const Triangle_Mesh_Geometry_Data* triangle_mesh_geometry_data)
{
    uint64_t mesh_hash = compute_triangle_mesh_hash(*triangle_mesh_geometry_data->mesh);
    if (geometry_data_hash != mesh_hash)
        return false;

    geometry_data = triangle_mesh_geometry_data;
    intersector = &intersect_triangle_mesh_geometry_data;
    return true;
}

bool KdTree::set_geometry_data(const Scene_Geometry_Data* scene_geometry_data)
{
    uint64_t scene_kdtree_data_hash = compute_scene_kdtree_data_hash(*scene_geometry_data);
    if (geometry_data_hash != scene_kdtree_data_hash)
        return false;

    geometry_data = scene_geometry_data;
    intersector = &intersect_scene_geometry_data;
    return true;
}

uint32_t KdTree::get_primitive_count() const
{
    if (intersector == &intersect_triangle_mesh_geometry_data)
        return static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data)->mesh->get_triangle_count();
    else
        return (uint32_t)static_cast<const Scene_Geometry_Data*>(geometry_data)->scene_objects->size();
}

uint64_t KdTree::get_allocated_memory_size() const
{
    return (uint64_t)(nodes.size() * sizeof(KdNode) + primitive_indices.size() * sizeof(uint32_t));
}

int KdTree::get_max_depth_limit(uint32_t primitive_count)
{
    int depth = std::lround(8.0 + 1.3 * std::floor(std::log2(primitive_count)));
    return std::min(depth, max_traversal_depth);
}

uint64_t KdTree::compute_triangle_mesh_hash(const Triangle_Mesh& mesh)
{
    // TODO: implement me
    return 0;
}

uint64_t KdTree::compute_scene_kdtree_data_hash(const Scene_Geometry_Data& scene_geometry_data)
{
    // TODO: implement me
    return 0;
}

//#define BRUTE_FORCE_INTERSECTION

bool KdTree::intersect(const Ray& ray, Intersection& intersection) const
{
#if USE_KD_TILES
    if (tile_buffer) {
        return intersect_tiled_structure(ray, intersection);
    }
#endif

#ifdef BRUTE_FORCE_INTERSECTION
    uint32_t primitive_count = 0;
    if (intersector == &intersect_triangle_mesh_kdtree_leaf_primitive)
        primitive_count = static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data)->mesh->get_triangle_count();
    else
        primitive_count = (uint32_t)static_cast<const Scene_KdTree_Data*>(geometry_data)->scene_objects->size();

    float tmax = intersection.t;
    for (uint32_t i = 0; i < primitive_count; i++) {
        intersector(ray, geometry_data, i, intersection);
    }
    return intersection.t < tmax;
#else
    float t_min, t_max; // parametric range for the ray's overlap with the current node

#if ENABLE_INVALID_FP_EXCEPTION
    if (!bounds.intersect_by_ray_without_NaNs(ray, &t_min, &t_max))
        return false;
#else
    if (!bounds.intersect_by_ray(ray, &t_min, &t_max))
        return false;
#endif

    struct Traversal_Info {
        const KdNode* node;
        float t_min;
        float t_max;
    };
    Traversal_Info traversal_stack[max_traversal_depth];
    int traversal_stack_size = 0;

    const KdNode* node = &nodes[0];
    const float ray_tmax = intersection.t;

    while (intersection.t > t_min) {
        if (!node->is_leaf()) {
            const int axis = node->get_split_axis();
            const float distance_to_split_plane = node->get_split_position() - ray.origin[axis];

            const KdNode* below_child = node + 1;
            const KdNode* above_child = &nodes[node->get_above_child()];

            // TODO: test this more on complex scenes. This improves performance on landscape scene (3.5%)
            // but for simpler scenes it's not always a win.
            //_mm_prefetch((const char*)above_child, _MM_HINT_T0);

            if (distance_to_split_plane != 0.0) { // general case
                const KdNode *first_child, *second_child;
                if (distance_to_split_plane > 0.0) {
                    first_child = below_child;
                    second_child = above_child;
                }
                else {
                    first_child = above_child;
                    second_child = below_child;
                }

                // Select node to traverse next.
                float t_split = distance_to_split_plane / ray.direction[axis]; // != 0 because distance_to_split_plane != 0
                if (t_split >= t_max || t_split < 0.0) {
                    node = first_child;
                }
                else if (t_split <= t_min) { // 0 < t_split <= t_min
                    node = second_child;
                }
                else { // t_min < t_split < t_max
                    ASSERT(traversal_stack_size < max_traversal_depth);
                    traversal_stack[traversal_stack_size++] = {second_child, t_split, t_max};
                    node = first_child;
                    t_max = t_split;
                }
            }
            else { // special case, distance_to_split_plane == 0.0
                if (ray.direction[axis] > 0.0)
                    node = above_child;
                else
                    node = below_child;
            }
        }
        else { // leaf node
            if (node->get_primitive_count() == 1) {
                intersector(ray, geometry_data, node->get_index(), intersection);
            }
            else {
                for (uint32_t i = 0; i < node->get_primitive_count(); i++) {
                    uint32_t primitive_index = primitive_indices[node->get_index() + i];
                    intersector(ray, geometry_data, primitive_index, intersection);
                }
            }

            if (traversal_stack_size == 0)
                break;

            --traversal_stack_size;

            node = traversal_stack[traversal_stack_size].node;
            t_min = traversal_stack[traversal_stack_size].t_min;
            t_max = traversal_stack[traversal_stack_size].t_max;
        }
    } // while (intersection.t > t_min)
    return intersection.t < ray_tmax;
#endif // !BRUTE_FORCE_INTERSECTION
}

#if USE_KD_TILES
bool KdTree::intersect_tiled_structure(const Ray& ray, Intersection& intersection) const
{
    float t_min, t_max; // parametric range for the ray's overlap with the current node

#if ENABLE_INVALID_FP_EXCEPTION
    if (!bounds.intersect_by_ray_without_NaNs(ray, &t_min, &t_max))
        return false;
#else
    if (!bounds.intersect_by_ray(ray, &t_min, &t_max))
        return false;
#endif

    struct Traversal_Info {
        const uint8_t* tile;
        const uint8_t* node;
        float t_min;
        float t_max;
        uint8_t primitive_count; // if > 0, then this traversal node contains leaf information
        uint32_t primitive_index;
    };
    Traversal_Info traversal_stack[max_traversal_depth];
    int traversal_stack_size = 0;

    const uint8_t* tile = tile_buffer.get();
    ASSERT(reinterpret_cast<uintptr_t>(tile_buffer.get()) % 64 == 0);
    const uint8_t* node = tile;
    const float ray_tmax = intersection.t;

    static constexpr int right_child_offset[4] = { 5, 10, 6, 9 };

    while (intersection.t > t_min) {
        uint8_t metadata = node[0];
        const int axis = metadata & 3;
        const float split_position = *reinterpret_cast<const float*>(node + 1);

        KdTile_Child_Type left_child_type = static_cast<KdTile_Child_Type>((metadata >> 2) & 3);
        KdTile_Child_Type right_child_type = static_cast<KdTile_Child_Type>((metadata >> 4) & 3);
        const uint8_t* left_child_info = node + 5;
        const uint8_t* right_child_info = node + right_child_offset[left_child_type];

        KdTile_Child_Type first_child_type;
        const uint8_t* first_child_info;
        KdTile_Child_Type second_child_type;
        const uint8_t* second_child_info;

        KdTile_Child_Type child_type;
        const uint8_t* child_info;

        const float distance_to_split_plane = split_position - ray.origin[axis];

        if (distance_to_split_plane != 0.0) { // general case
            if (distance_to_split_plane > 0.f) {
                first_child_type = left_child_type;
                first_child_info = left_child_info;
                second_child_type = right_child_type;
                second_child_info = right_child_info;
            }
            else {
                first_child_type = right_child_type;
                first_child_info = right_child_info;
                second_child_type = left_child_type;
                second_child_info = left_child_info;
            }

            // Select node to traverse next.
            float t_split = distance_to_split_plane / ray.direction[axis]; // != 0 because distance_to_split_plane != 0
            if (t_split >= t_max || t_split < 0.0) {
                child_type = first_child_type;
                child_info = first_child_info;
            }
            else if (t_split <= t_min) { // 0 < t_split <= t_min
                child_type = second_child_type;
                child_info = second_child_info;
            }
            else { // t_min < t_split < t_max
                ASSERT(traversal_stack_size < max_traversal_depth);

                if (second_child_type == kdtile_child_type_node) {
                    traversal_stack[traversal_stack_size++] = { tile, tile + second_child_info[0], t_split, t_max, 0, 0 };
                }
                else if (second_child_type == kdtile_child_type_external_node) {
                    uint32_t cache_line_index = *reinterpret_cast<const uint32_t*>(second_child_info);
                    const uint8_t* external_tile = tile_buffer.get() + cache_line_index * cache_line_size;
                    traversal_stack[traversal_stack_size++] = { external_tile, external_tile, t_split, t_max, 0, 0 };
                }
                else if (second_child_type == kdtile_child_type_leaf) {
                    uint8_t primitive_count = second_child_info[0];
                    uint32_t primitive_offset = *reinterpret_cast<const uint32_t*>(second_child_info + 1);
                    traversal_stack[traversal_stack_size++] = { nullptr, nullptr, 0.f, 0.f, primitive_count, primitive_offset };
                }
                child_type = first_child_type;
                child_info = first_child_info;
                t_max = t_split;
            }
        }
        else { // special case, distance_to_split_plane == 0.0
            if (ray.direction[axis] > 0.0) {
                child_type = right_child_type;
                child_info = right_child_info;
            }
            else {
                child_type = left_child_type;
                child_info = left_child_info;
            }
        }

        if (child_type == kdtile_child_type_node) {
            node = tile + child_info[0];
        }
        else if (child_type == kdtile_child_type_external_node) {
            uint32_t cache_line_index = *reinterpret_cast<const uint32_t*>(child_info);
            tile = tile_buffer.get() + cache_line_index * cache_line_size;
            node = tile;
        }
        else { // kdtile_child_type_leaf or kdtile_child_type_empty
            uint8_t primitive_count = 0;
            uint32_t primitive_offset;

            if (child_type == kdtile_child_type_leaf) {
                primitive_count = child_info[0];
                primitive_offset = *reinterpret_cast<const uint32_t*>(child_info + 1);
            }
            do {
                if (primitive_count == 1) {
                    intersector(ray, geometry_data, primitive_offset, intersection);
                }
                else {
                    for (uint32_t i = 0; i < primitive_count; i++) {
                        uint32_t primitive_index = primitive_indices[primitive_offset + i];
                        intersector(ray, geometry_data, primitive_index, intersection);
                    }
                }

                if (traversal_stack_size == 0)
                    goto traversal_finished;

                --traversal_stack_size;

                tile = traversal_stack[traversal_stack_size].tile;
                node = traversal_stack[traversal_stack_size].node;
                t_min = traversal_stack[traversal_stack_size].t_min;
                t_max = traversal_stack[traversal_stack_size].t_max;
                primitive_count = traversal_stack[traversal_stack_size].primitive_count;
                primitive_offset = traversal_stack[traversal_stack_size].primitive_index;
            } while (primitive_count > 0);
        }
    } // while (intersection.t > t_min)
traversal_finished:
    return intersection.t < ray_tmax;
}
#endif // USE_KD_TILES

bool KdTree::intersect_any(const Ray& ray, float tmax) const
{
    Intersection intersection{ tmax };
    return intersect(ray, intersection);
}
