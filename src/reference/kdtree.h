#pragma once

#include "intersection.h"
#include "triangle_mesh.h"
#include "lib/bounding_box.h"
#include "lib/common.h"
#include "lib/ray.h"
#include "lib/vector.h"

#include <cassert>
#include <cstdint>
#include <vector>

struct Local_Geometry;
struct Triangle_Intersection;
struct Mesh_Source;
struct KdTree_Source;

template <typename Primitive_Source> class KdTree;

using Mesh_KdTree       = KdTree<Mesh_Source>;
using TwoLevel_KdTree   = KdTree<KdTree_Source>;

// Contains data collected by KdTree::calculate_stats() function.
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

// 8 byte kd-tree node.
// KdTree is a linear array of nodes + an array of triangle indices referenced by the nodes.
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

// KdTree acceleration structure.
// Template parameter specifies the object that provides primitives to build kdtree from.
template <typename Primitive_Source>
class KdTree {
public:
    KdTree() = default;
    KdTree(KdTree&& other) = default;
    KdTree& operator=(KdTree&& other) = default;

    float intersect(const Ray& ray, Local_Geometry& local_geom) const;

    const Primitive_Source& get_primitive_source() const { return primitive_source; }
    const Bounding_Box& get_bounds() const { return bounds; }

    KdTree_Stats calculate_stats() const;
    std::vector<int32_t> calculate_path_to_node(int32_t node_index) const;

    void save_to_file(const std::string& file_name) const;

private:
    void intersect(const Ray& ray, Triangle_Intersection& intersection) const;
    void intersect_leaf(const Ray& ray, KdNode leaf, Triangle_Intersection& intersection) const;

private:
    KdTree(std::vector<KdNode>&& nodes, std::vector<int32_t>&& triangle_indices, const Primitive_Source& primitive_source);

    template <typename Primitive_Source> friend class KdTree_Builder;
    friend struct KdTree_Source;
    friend KdTree<Mesh_Source> load_mesh_kdtree(const std::string& file_name, const Triangle_Mesh& mesh);

    enum { max_traversal_depth = 64 };

private:
    std::vector<KdNode>   nodes;
    std::vector<int32_t>  triangle_indices;
    Primitive_Source      primitive_source;
    Bounding_Box          bounds;
};

Mesh_KdTree load_mesh_kdtree(const std::string& file_name, const Triangle_Mesh& mesh);

struct Mesh_Source {
    const Triangle_Mesh* mesh;

    Mesh_Source() {}
    Mesh_Source(const Triangle_Mesh* mesh) : mesh(mesh) {}

    int32_t get_primitive_count() const {
        return mesh->get_triangle_count();
    }
    Bounding_Box get_primitive_bounds(int32_t primitive_index) const {
        return mesh->get_triangle_bounds(primitive_index);
    }
    Bounding_Box calculate_bounds() const {
        return mesh->get_bounds();
    }
    void intersect(const Ray& ray, int32_t primitive_index, Triangle_Intersection& intersection) const {
        return intersect_triangle(ray, mesh, primitive_index, intersection);
    }
};

struct KdTree_Source {
    std::vector<const Mesh_KdTree*> mesh_kdtrees;

    int32_t get_primitive_count()  const {
        return (int32_t)mesh_kdtrees.size();
    }
    Bounding_Box get_primitive_bounds(int32_t primitive_index) const {
        assert(primitive_index >= 0 && primitive_index < mesh_kdtrees.size());
        return mesh_kdtrees[primitive_index]->get_bounds();
    }
    Bounding_Box calculate_bounds() const {
        Bounding_Box bounds;
        for (auto kdtree : mesh_kdtrees) {
            bounds = Bounding_Box::get_union(bounds, kdtree->get_bounds());
        }
        return bounds;
    }
    void intersect(const Ray& ray, int32_t primitive_index, Triangle_Intersection& intersection) const {
        assert(primitive_index >= 0 && primitive_index < mesh_kdtrees.size());
        mesh_kdtrees[primitive_index]->intersect(ray, intersection);

    }
};
