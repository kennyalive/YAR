#pragma once

#include "lib/bounding_box.h"
#include "lib/geometry.h"

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

    void init_leaf(uint32_t primitive_count) {
        word0 = leaf_or_axis_mask | (primitive_count << 2);
        // Primitive indices are stored in the array with the first element in the
        // word1 field and other elements are stored in memory sequentially.
        // The array is padded to end on KdNode boundary.
        word1 = uint32_t(-1);
    }

    bool is_leaf() const {
        return (word0 & leaf_or_axis_mask) == leaf_or_axis_mask;
    }

    uint32_t get_primitive_count() const {
        ASSERT(is_leaf());
        return word0 >> 2;
    }

    const uint32_t* get_primitive_indices_array() const {
        ASSERT(is_leaf());
        return &word1;
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
