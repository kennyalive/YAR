#pragma once

#include "lib/bounding_box.h"
#include "lib/geometry.h"

#define USE_KD_TILES 1

#if USE_KD_TILES
#include "kdtile.h"
#endif

struct Intersection;
struct KdTree;
struct Ray;
struct Scene_Object;
class Image_Texture;

// 8 byte kd-tree node.
// KdTree is a linear array of nodes + an array of primitive indices referenced by the nodes.
struct KdNode {
    uint32_t word0;
    uint32_t word1;

    // 30 most significant bits of word0 of interior node is the above child node index.
    static constexpr uint32_t max_node_count = 0x40000000; // max ~1 billion nodes.

    // 2 least significant bits of word0. If both are set (3) it's a leaf node,
    // otherwise it's an axis index [0..2] for interior node
    static constexpr uint32_t leaf_or_axis_mask = 3;

    void init_interior_node(int axis, uint32_t above_child, float split) {
        // 0 - x axis, 1 - y axis, 2 - z axis
        ASSERT(axis >= 0 && axis < 3);
        ASSERT(above_child < max_node_count);

        word0 = uint32_t(axis) | (above_child << 2);
        word1 = std::bit_cast<uint32_t>(split);
    }

    void init_empty_node() { // leaf without primitives
        word0 = leaf_or_axis_mask; // word0 == 3
        word1 = 0; // not used for empty node, just sets default value
    }

    void init_leaf_with_single_primitive(uint32_t primitive_index) {
        word0 = leaf_or_axis_mask | (1 << 2); // word0 == 7
        word1 = primitive_index;
    }

    void init_leaf_with_multiple_primitives(uint32_t primitive_count, uint32_t primitive_indices_offset) {
        ASSERT(primitive_count > 1);
        // word0 == 11, 15, 19, ... (for primitive_count = 2, 3, 4, ...)
        word0 = leaf_or_axis_mask | (primitive_count << 2);
        word1 = primitive_indices_offset;
    }

    //
    // Leaf node queries.
    //
    bool is_leaf() const {
        return (word0 & leaf_or_axis_mask) == leaf_or_axis_mask;
    }

    bool is_empty() const {
        return is_leaf() && get_primitive_count() == 0;
    }

    uint32_t get_primitive_count() const {
        ASSERT(is_leaf());
        return word0 >> 2;
    }

    uint32_t get_index() const {
        ASSERT(is_leaf());
        return word1;
    }

    //
    // Interior node queries.
    //
    bool is_interior() const {
        return !is_leaf();
    }

    int get_split_axis() const {
        ASSERT(!is_leaf());
        return int(word0 & leaf_or_axis_mask);
    }

    float get_split_position() const {
        ASSERT(!is_leaf());
        return std::bit_cast<float>(word1);
    }

    uint32_t get_above_child() const {
        ASSERT(!is_leaf());
        return word0 >> 2;
    }
};

struct Triangle_Mesh_Geometry_Data {
    const Triangle_Mesh* mesh = nullptr;
    const Image_Texture* alpha_texture = nullptr;
};

// Data used by top level (entire scene) kdtree.
struct Scene_Geometry_Data {
    // The objects in the scene. Each object has associated geometry kdtree.
    const std::vector<Scene_Object>* scene_objects = nullptr;

    // KdTrees for all geometries in the scene.
    // Multiple scene objects can use the same kdtree due to instancing.
    const std::vector<KdTree>* kdtrees = nullptr;

    // Offsets in kdtrees array.
    // Each offset defines where the range of kdtrees for specific geometry type starts.
    std::array<int, Geometry_Type_Count> geometry_type_offsets;
};

struct KdTree {
    mutable uint64_t inside_tile_transitions = 0;
    mutable uint64_t external_tile_transitions = 0;

    static KdTree load(const std::string& file_name);
    void save(const std::string& file_name) const;

    // Sets reference to geometry data. Should be called after kdtree is loaded from the file.
    // These functions return false when the hash computed from geometry data differs from KdTree::geometry_data_hash.
    bool set_geometry_data(const Triangle_Mesh_Geometry_Data* triangle_mesh_geometry_data);
    bool set_geometry_data(const Scene_Geometry_Data* scene_geometry_data);

    uint32_t get_primitive_count() const;
    uint64_t get_allocated_memory_size() const;

    static int get_max_depth_limit(uint32_t primitive_count);
    static uint64_t compute_triangle_mesh_hash(const Triangle_Mesh& mesh);
    static uint64_t compute_scene_kdtree_data_hash(const Scene_Geometry_Data& scene_geometry_data);

    bool intersect(const Ray& ray, Intersection& intersection) const;
    bool intersect_any(const Ray& ray, float tmax) const;
    
    Bounding_Box bounds; // kdtree spatial bounds

    // Hash of the geometry data this kdtree was originally created from.
    // It is used to invalidate cached kdtree when geometry changes.
    uint64_t geometry_data_hash = 0;

    std::vector<KdNode> nodes;
    std::vector<uint32_t> primitive_indices;

    struct Tile_Buffer_Deleter {
        void operator()(uint8_t* data) {
            _aligned_free(data);
        }
    };
    std::unique_ptr<uint8_t, Tile_Buffer_Deleter> tile_buffer; // cache line aligned
    size_t tile_buffer_size = 0;

    // Reference to geometry data for which this kdtree is built.
    // For triangle mesh kdtree it points to Triangle_Mesh_Geometry_Data object.
    // For scene kdtree it points to Scene_Geometry_Data object.  
    const void* geometry_data = nullptr;

    // Performs intersection test between a ray and a primitive from kdtree leaf.
    // 
    // The ray's parametric range is from the half-open interval [0, t_max), where
    // t_max is the initial value of Intersection::t. If intersection is found then
    // Intersection::t gets overwritten with a distance to the intersection point,
    // otherwise Intersection::t is unchanged.
    void (*intersector)(const Ray& ray, const void* geometry_data, uint32_t primitive_index, Intersection& intersection) = nullptr;
};
