#pragma once

#include "intersection.h"
#include "lib/bounding_box.h"
#include "lib/common.h"
#include "lib/matrix.h"
#include "lib/ray.h"
#include "lib/render_object.h"
#include "lib/triangle_mesh.h"
#include "lib/vector.h"

#include <cstdint>
#include <vector>

struct Local_Geometry;

template <typename Primitive_Source> class KdTree;

struct Geometry_Primitive_Source;
using Geometry_KdTree = KdTree<Geometry_Primitive_Source>;

struct KdTreeList_Primitive_Source;
using Scene_KdTree = KdTree<KdTreeList_Primitive_Source>;

// Contains data collected by KdTree::calculate_stats() function.
struct KdTree_Stats {
    int64_t nodes_size = 0;
    int64_t primitive_indices_size = 0;

    int32_t node_count = 0;
    int32_t leaf_count = 0;
    int32_t empty_leaf_count = 0;
    int32_t single_primitive_leaf_count = 0;
    int     perfect_depth = 0;

    struct Leaf_Stats {
        float average_depth = 0.0f;
        float depth_standard_deviation = 0.0f;
        float average_primitive_count = 0.0f;
    };
    Leaf_Stats not_empty_leaf_stats;
    Leaf_Stats empty_leaf_stats; // empty_leaf_stats.average_primitive_count == 0

    void print();
};

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

// KdTree acceleration structure.
// Template parameter specifies the object that provides primitives to build kdtree from.
template <typename Primitive_Source>
class KdTree {
public:
    KdTree() = default;
    KdTree(KdTree&& other) = default;
    KdTree& operator=(KdTree&& other) = default;

    float intersect(const Ray& ray, Local_Geometry& local_geom) const;
    float intersect_any(const Ray& ray) const;

    const Primitive_Source& get_primitive_source() const { return primitive_source; }
    const Bounding_Box& get_bounds() const { return bounds; }
    KdTree_Stats calculate_stats() const;
    std::vector<int32_t> calculate_path_to_node(int32_t node_index) const;
    void save_to_file(const std::string& file_name) const;

private:
    void intersect(const Ray& ray, Intersection& intersection) const;
    void intersect_leaf(const Ray& ray, KdNode leaf, Intersection& intersection) const;

private:
    KdTree(std::vector<KdNode>&& nodes, std::vector<int32_t>&& primitive_indices, const Primitive_Source& primitive_source);

    template <typename Primitive_Source> friend class KdTree_Builder;
    friend struct KdTreeList_Primitive_Source;
    friend Geometry_KdTree load_geometry_kdtree(const std::string& file_name, const Geometries* geometries, Geometry_Handle hgeometry);

    enum { max_traversal_depth = 64 };

private:
    std::vector<KdNode>   nodes;
    std::vector<int32_t>  primitive_indices;
    Primitive_Source      primitive_source;
    Bounding_Box          bounds;
};

Geometry_KdTree load_geometry_kdtree(const std::string& file_name, const Geometries* geometries, Geometry_Handle hgeometry);

struct Geometry_Primitive_Source {
    const Geometries* geometries;
    Geometry_Handle hgeometry;
    Matrix3x4 world_to_mesh_transform;

    Geometry_Primitive_Source() {}
    Geometry_Primitive_Source(const Geometries* geometries, Geometry_Handle hgeometry)
        : geometries(geometries)
        , hgeometry(hgeometry)
    {}

    int32_t get_primitive_count() const {
        if (hgeometry.type == Geometry_Type::triangle_mesh) {
            return geometries->triangle_meshes[hgeometry.index].get_triangle_count();
        }
        else {
            ASSERT(false);
            return 0;
        }
    }

    Bounding_Box get_primitive_bounds(int32_t primitive_index) const {
        if (hgeometry.type == Geometry_Type::triangle_mesh) {
            return geometries->triangle_meshes[hgeometry.index].get_triangle_bounds(primitive_index);
        }
        else {
            ASSERT(false);
            return Bounding_Box{};
        }
    }

    Bounding_Box calculate_bounds() const {
        if (hgeometry.type == Geometry_Type::triangle_mesh) {
            return geometries->triangle_meshes[hgeometry.index].get_bounds();
        }
        else {
            ASSERT(false);
            return Bounding_Box{};
        }
    }

    void intersect(const Ray& ray, int32_t primitive_index, Intersection& intersection) const {
        if (hgeometry.type == Geometry_Type::triangle_mesh) {
            intersect_triangle(ray, &geometries->triangle_meshes[hgeometry.index], primitive_index, intersection);
        }
        else {
            ASSERT(false);
        }
    }
};

struct KdTreeList_Primitive_Source {
    std::vector<const Geometry_KdTree*> geometry_kdtrees;

    int32_t get_primitive_count()  const {
        return (int32_t)geometry_kdtrees.size();
    }

    Bounding_Box get_primitive_bounds(int32_t primitive_index) const {
        ASSERT(primitive_index >= 0 && primitive_index < geometry_kdtrees.size());
        return geometry_kdtrees[primitive_index]->get_bounds();
    }

    Bounding_Box calculate_bounds() const {
        Bounding_Box bounds;
        for (auto kdtree : geometry_kdtrees)
            bounds = Bounding_Box::get_union(bounds, kdtree->get_bounds());
        return bounds;
    }

    void intersect(const Ray& ray, int32_t primitive_index, Intersection& intersection) const {
        ASSERT(primitive_index >= 0 && primitive_index < geometry_kdtrees.size());
        geometry_kdtrees[primitive_index]->intersect(ray, intersection);
    }
};

