#pragma once

#include "lib/bounding_box.h"
#include "lib/geometry.h"

struct Intersection;
struct KdTree;
struct Ray;
struct Scene_Object;

// 8 byte kd-tree node.
// KdTree is a linear array of nodes + an array of primitive indices referenced by the nodes.
struct KdNode {
    uint32_t word0;
    uint32_t word1;

    enum : int32_t { max_node_count = 0x40000000 }; // max ~ 1 billion nodes
    enum : uint32_t { leaf_node_flags = 3 };

    void init_interior_node(int axis, int32_t above_child, float split) {
        // 0 - x axis, 1 - y axis, 2 - z axis
        ASSERT(axis >= 0 && axis < 3);
        ASSERT(above_child < max_node_count);

        word0 = axis | (static_cast<uint32_t>(above_child) << 2);
        word1 = *reinterpret_cast<uint32_t*>(&split);
    }

    void init_empty_leaf() {
        word0 = leaf_node_flags; // word0 == 3
        word1 = 0; // not used for empty leaf, just sets default value
    }

    void init_leaf_with_single_primitive(int32_t primitive_index) {
        word0 = leaf_node_flags | (1 << 2); // word0 == 7
        word1 = static_cast<uint32_t>(primitive_index);
    }

    void init_leaf_with_multiple_primitives(int32_t primitive_count, int32_t primitive_indices_offset) {
        ASSERT(primitive_count > 1);
        // word0 == 11, 15, 19, ... (for primitive_count = 2, 3, 4, ...)
        word0 = leaf_node_flags | (static_cast<uint32_t>(primitive_count) << 2);
        word1 = static_cast<uint32_t>(primitive_indices_offset);
    }

    bool is_leaf() const {
        return (word0 & leaf_node_flags) == leaf_node_flags;
    }

    int32_t get_primitive_count() const {
        ASSERT(is_leaf());
        return static_cast<int32_t>(word0 >> 2);
    }

    int32_t get_index() const {
        ASSERT(is_leaf());
        return static_cast<int32_t>(word1);
    }

    int get_split_axis() const {
        ASSERT(!is_leaf());
        return static_cast<int>(word0 & leaf_node_flags);
    }

    float get_split_position() const {
        ASSERT(!is_leaf());
        return *reinterpret_cast<const float*>(&word1);
    }

    int32_t get_above_child() const {
        ASSERT(!is_leaf());
        return static_cast<int32_t>(word0 >> 2);
    }
};

struct KdTree_Stats {
    int64_t nodes_size = 0;
    int64_t primitive_indices_size = 0;

    int node_count = 0;
    int leaf_count = 0;
    int empty_leaf_count = 0;
    int single_primitive_leaf_count = 0;
    int perfect_depth = 0;

    struct Leaf_Stats {
        float average_depth = 0.0f;
        float depth_standard_deviation = 0.0f;
        float average_primitive_count = 0.0f;
    };
    Leaf_Stats not_empty_leaf_stats;
    Leaf_Stats empty_leaf_stats; // empty_leaf_stats.average_primitive_count == 0

    void print();
};

// Data used by top level (entire scene) kdtree.
struct Scene_KdTree_Data {
    // The objects in the scene. Each object has associated geometry kdtree.
    const std::vector<Scene_Object>* scene_objects = nullptr;

    // KdTrees for all geometries in the scene.
    // Multiple scene objects can use the same kdtree due to instancing.
    const std::vector<KdTree>* kdtrees = nullptr;

    // Offsets in 'kdtrees' array for trees associated with specific geometry type.
    std::array<int, Geometry_Type_Count> geometry_type_offsets;
};

struct KdTree {
    static KdTree load(const std::string& file_name);
    void save(const std::string& file_name) const;

    // Sets reference to geometry data. Should be called after kdtree is loaded from the file.
    // These functions return false when the hash computed from geometry data differs from KdTree::geometry_data_hash.
    bool set_geometry_data(const Triangle_Mesh* mesh);
    bool set_geometry_data(const Scene_KdTree_Data* scene_kdtree_data);

    static uint64_t compute_triangle_mesh_hash(const Triangle_Mesh& mesh);
    static uint64_t compute_scene_kdtree_data_hash(const Scene_KdTree_Data& data);

    KdTree_Stats calculate_stats() const;
    std::vector<int32_t> calculate_path_to_node(int32_t node_index) const;

    static constexpr int max_traversal_depth = 40;
    bool intersect(const Ray& ray, Intersection& intersection) const;
    bool intersect_any(const Ray& ray, float tmax) const;
    
    Bounding_Box bounds; // kdtree spatial bounds

    // Hash of the geometry data this kdtree was originally created from.
    // It is used to invalidate cached kdtree when geometry changes.
    uint64_t geometry_data_hash = 0;

    std::vector<KdNode> nodes;
    std::vector<int> primitive_indices;

    // Reference to geometry data for which this kdtree is built.
    //
    // For triangle mesh kdtree it's a Triangle_Mesh object.
    // The leaves contain references to seperate triangles.
    //
    // For scene kdtree it's a Scene_KdTree_Data object.  
    // The leaves contain references to scene objects.
    const void* geometry_data = nullptr;

    // Performs intersection test between a ray and a primitive from kdtree leaf.
    // 
    // The ray's parametric range is from the half-open interval [0, t_max), where
    // t_max is the initial value of Intersection::t. If intersection is found then
    // Intersection::t gets overwritten with a distance to the intersection point,
    // otherwise Intersection::t is unchanged.
    void (*intersector)(const Ray& ray, const void* geometry_data, int primitive_index, Intersection& intersection) = nullptr;
};
