#pragma once

#include "common.h"
#include "bounding_box.h"
#include "ray.h"
#include "triangle.h"
#include "triangle_mesh.h"
#include "vector.h"

#include <cassert>
#include <cstdint>
#include <vector>

struct KdTree_Stats {
    int64_t nodes_size = 0;
    int64_t triangle_indices_size = 0;

    int32_t node_count = 0;
    int32_t leaf_count = 0;
    int32_t empty_leaf_count = 0;
    int32_t single_triangle_leaf_count = 0;
    int     perfect_depth = 0;

    struct Leaf_Stats {
        float average_depth = 0.0f;
        float depth_standard_deviation = 0.0f;
        float average_triangle_count = 0.0f;
    };
    Leaf_Stats not_empty_leaf_stats;
    Leaf_Stats empty_leaf_stats; // empty_leaf_stats.average_triangle_count == 0

    void print();
};

struct KdNode {
    uint32_t word0;
    uint32_t word1;

    enum : int32_t { max_node_count = 0x40000000 }; // max ~ 1 billion nodes
    enum : uint32_t { leaf_node_flags = 3 };

    void init_interior_node(int axis, int32_t above_child, float split) {
        // 0 - x axis, 1 - y axis, 2 - z axis
        assert(axis >= 0 && axis < 3);
        assert(above_child < max_node_count);

        word0 = axis | (static_cast<uint32_t>(above_child) << 2);
        word1 = *reinterpret_cast<uint32_t*>(&split);
    }

    void init_empty_leaf() {
        word0 = leaf_node_flags; // word0 == 3
        word1 = 0;             // not used for empty leaf, just set default value
    }

    void init_leaf_with_single_triangle(int32_t triangle_index) {
        word0 = leaf_node_flags | (1 << 2); // word0 == 7
        word1 = static_cast<uint32_t>(triangle_index);
    }

    void init_leaf_with_multiple_triangles(int32_t triangle_count, int32_t triangle_indices_offset) {
        assert(triangle_count > 1);
        // word0 == 11, 15, 19, ... (for numTriangles = 2, 3, 4, ...)
        word0 = leaf_node_flags | (static_cast<uint32_t>(triangle_count) << 2);
        word1 = static_cast<uint32_t>(triangle_indices_offset);
    }

    bool is_leaf() const {
        return (word0 & leaf_node_flags) == leaf_node_flags;
    }

    bool is_interior_node() const {
        return !is_leaf();
    }

    int32_t get_triangle_count() const {
        assert(is_leaf());
        return static_cast<int32_t>(word0 >> 2);
    }

    int32_t get_index() const {
        assert(is_leaf());
        return static_cast<int32_t>(word1);
    }

    int get_split_axis() const {
        assert(is_interior_node());
        return static_cast<int>(word0 & leaf_node_flags);
    }

    float get_split_position() const {
        assert(is_interior_node());
        return *reinterpret_cast<const float*>(&word1);
    }

    int32_t get_above_child() const {
        assert(is_interior_node());
        return static_cast<int32_t>(word0 >> 2);
    }
};

class KdTree {
public:
    struct Intersection {
        float t = Infinity;
        float epsilon = 0.0;
    };

public:
    KdTree(std::vector<KdNode>&& nodes, std::vector<int32_t>&& triangle_indices, const Triangle_Mesh& mesh);
    KdTree(const std::string& file_name, const Triangle_Mesh& mesh);

    void save_to_file(const std::string& file_name) const;

    bool intersect(const Ray& ray, Intersection& intersection) const;

    const Triangle_Mesh& get_mesh() const { return mesh; }
    KdTree_Stats calculate_stats() const;
    std::vector<int32_t> calculate_path_to_node(int32_t node_index) const;

private:
    void intersect_leaf_triangles(const Ray& ray, KdNode leaf, Triangle_Intersection& closest_intersection) const;

private:
    friend class KdTree_Builder;

    enum { max_traversal_depth = 64 };

private:
    const std::vector<KdNode>   nodes;
    const std::vector<int32_t>  triangle_indices;
    const Triangle_Mesh&        mesh;
    const Bounding_Box          mesh_bounds;
};
