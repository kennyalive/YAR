#include "std.h"
#include "lib/common.h"
#include "lib/triangle_mesh.h"
#include "kdtree_builder.h"

namespace {

struct Edge {
    float position_on_axis;
    uint32_t primitive_and_flags;

    enum : uint32_t { edge_end_flag = 0x80000000 };
    enum : uint32_t { primitive_perpendicular_to_axis_flag = 0x40000000 };
    enum : uint32_t { primitive_mask = 0x3fffffff };

    bool is_start() const {
        return (primitive_and_flags & edge_end_flag) == 0;
    }

    bool is_end() const {
        return !is_start();
    }

    bool is_primitive_perpendicular_to_axis() const {
        return (primitive_and_flags & primitive_perpendicular_to_axis_flag) != 0;
    }

    int32_t get_primitive_index() const {
        return static_cast<int32_t>(primitive_and_flags & primitive_mask);
    }

    static bool less(Edge edge1, Edge edge2) {
        if (edge1.position_on_axis == edge2.position_on_axis)
            return edge1.is_end() && edge2.is_start();
        else
            return edge1.position_on_axis < edge2.position_on_axis;
    }
};

struct Split {
    int32_t edge;
    int axis;
    float cost;
};

struct Primitive_Info {
	int32_t primitive;
	Bounding_Box bounds;
};
} // namespace

template <typename Primitive_Source>
class KdTree_Builder {
public:
    KdTree_Builder(Primitive_Source&& primitive_source, const KdTree_Build_Params& build_params);
    KdTree<Primitive_Source> build();

private:
    void build_node(const Bounding_Box& node_bounds, int32_t primitives_offset, int32_t primitive_count,
        int depth, int32_t above_primitive_offset);

    void create_leaf(const Primitive_Info* primitives, int32_t primitive_count);
    Split select_split(const Bounding_Box& node_bounds, const Primitive_Info* primitives, int32_t primitive_count);
    Split select_split_for_axis(const Bounding_Box& node_bounds, int32_t primitive_count, int axis) const;

private:
    Primitive_Source&& primitive_source;
    KdTree_Build_Params build_params;

    std::vector<Edge> edges[3]; // edges for each axis
    std::vector<Primitive_Info> primitive_buffer;
    std::vector<Primitive_Info> primitive_buffer2;

    std::vector<KdNode> nodes;
    std::vector<int32_t> primitive_indices;
};

Geometry_KdTree build_geometry_kdtree(const Geometries* geometries, Geometry_Handle hgeometry, const KdTree_Build_Params& build_params) {
    Geometry_Primitive_Source primitive_source {geometries, hgeometry};
    KdTree_Builder<Geometry_Primitive_Source> builder(std::move(primitive_source), build_params);
    return builder.build();
}

Scene_KdTree build_scene_kdtree(const Scene* scene, std::vector<Geometry_KdTree>&& kdtrees, const KdTree_Build_Params& build_params) {
    Scene_Primitive_Source primitive_source;
    primitive_source.scene = scene;
    primitive_source.geometry_kdtrees = std::move(kdtrees);
    KdTree_Builder<Scene_Primitive_Source> builder(std::move(primitive_source), build_params);
    return builder.build();
}

enum {
  // max count is chosen such that max_primitive_count * 2 is still an int32_t,
  // this simplifies implementation.
  max_primitive_count = 0x3fffffff // max ~ 1 billion primitives
};

template <typename Primitive_Source>
KdTree_Builder<Primitive_Source>::KdTree_Builder(Primitive_Source&& primitive_source, const KdTree_Build_Params& build_params)
: primitive_source(std::move(primitive_source))
, build_params(build_params)
{
    if (primitive_source.get_primitive_count() > max_primitive_count) {
        error("exceeded the maximum number of primitives: " + std::to_string(max_primitive_count));
    }

    if (this->build_params.max_depth <= 0) {
        this->build_params.max_depth = std::lround(8.0 + 1.3 * std::floor(std::log2(primitive_source.get_primitive_count())));
    }
    this->build_params.max_depth = std::min(this->build_params.max_depth, static_cast<int>(KdTree<Primitive_Source>::max_traversal_depth));
}

template <typename Primitive_Source>
KdTree<Primitive_Source> KdTree_Builder<Primitive_Source>::build()
{
    const int32_t primitive_count = primitive_source.get_primitive_count();

    //
    // Prepare working structures.
    //
    for (int i = 0; i < 3; i++)
        edges[i].resize(2 * primitive_count);

    // Maximum theoretical size of primitive is [primitive_count*(build_params.max_depth + 1)] elements.
    // Algorithm starts with the following initial size and resizes later if necessary.
    primitive_buffer.resize(static_cast<size_t>(primitive_count * 2.5));

    primitive_buffer2.resize(primitive_count);

    Bounding_Box total_bounds;
    for (int32_t i = 0; i < primitive_count; i++) {
        Bounding_Box bounds = primitive_source.get_primitive_bounds(i);
        primitive_buffer[i] = { i, bounds };
        total_bounds = Bounding_Box::compute_union(total_bounds, bounds);
    }

    //
    // Recursively build all nodes.
    //
    build_node(total_bounds, 0, primitive_count, build_params.max_depth, primitive_count);

    return KdTree<Primitive_Source>(std::move(nodes), std::move(primitive_indices), std::move(primitive_source));
}

//
// Splits the provided bounding box and selects either left or right part based on the provided boolean flag.
// The selected part is additionally clipped to be as tight as possible taking into account triangle geometry.
// Implements clipping algorithm described in this article:
//      Alexei Soupikov, Maxim Shevtsov, Alexander Kapustin, 2008. Improving Kd-tree Quality at a Reasonable Construction Cost
//
void clip_bounds(const Geometry_Primitive_Source& primitive_source, int32_t triangle_index, float split_position, int axis, bool left, Bounding_Box& bounds)
{
    ASSERT(split_position > bounds.min_p[axis] && split_position < bounds.max_p[axis]);

    if (left)
        bounds.max_p[axis] = split_position;
    else
        bounds.min_p[axis] = split_position;

    if (primitive_source.geometry.type != Geometry_Type::triangle_mesh)
        return;

    // sort triangle vertices along the split dimension
    Vector3 p[3];
    primitive_source.geometries->triangle_meshes[primitive_source.geometry.index].get_triangle(triangle_index, p[0], p[1], p[2]);

    if (p[1][axis] < p[0][axis])
        std::swap(p[1], p[0]);

    if (p[2][axis] < p[0][axis])
        std::swap(p[2], p[0]);

    if (p[2][axis] < p[1][axis])
        std::swap(p[2], p[1]);

    // re-index sorted points:
    // A is a common vertex of 2 edges intersected by a splitting plane,
    // B is a middle vertex,
    // C is the remaining third vertex
    bool middle_on_the_left = p[1][axis] < split_position;

    Vector3 a, c;
    Vector3 b = p[1];
    if (middle_on_the_left) {
        a = p[2];
        c = p[0];
    } else {
        a = p[0];
        c = p[2];
    }

    // find insersection points of two edges with a splitting plane
    Vector3 isect_ab, isect_ac;
    {
        // This epsilon deals with the floating point imprecision and avoids the use case
        // when calculated intersection point does not reach split_position plane.
        // The above issue leads to the nodes that might not include some triangles that intersect them.
        float epsilon = (left && !middle_on_the_left || !left && middle_on_the_left) ? 1e-5f : -1e-5f;

        Vector3 ab = b - a;
        isect_ab = a + ab * ((split_position - a[axis]) / ab[axis] + epsilon);

        Vector3 ac = c - a;
        isect_ac = a + ac * ((split_position - a[axis]) / ac[axis] + epsilon);

        // Ensure that epsilon provides enough offset.
        // If any of the asserts signal then epsilon should be increased.
        // Larger epsilon is not a problem for the algorithm, it's not sensitive to the accuracy of the
        // intersection point calculation, only to the fact whether it reaches the splitting plane or not.
        ASSERT(left && isect_ab[axis] >= split_position || !left && isect_ab[axis] <= split_position);
        ASSERT(left && isect_ac[axis] >= split_position || !left && isect_ac[axis] <= split_position);
    }

    // construct bounding box
    Bounding_Box bounds2;
    bounds2.add_point(isect_ab);
    bounds2.add_point(isect_ac);

    if (left) {
        bounds2.add_point(p[0]);
        if (middle_on_the_left)
            bounds2.add_point(p[1]);
    } else {
        bounds2.add_point(p[2]);
        if (!middle_on_the_left)
            bounds2.add_point(p[1]);
    }
    bounds = Bounding_Box::compute_intersection(bounds, bounds2);
}

void clip_bounds(const Scene_Primitive_Source&, int32_t, float split_position, int axis, bool left, Bounding_Box& bounds) {
    ASSERT(split_position > bounds.min_p[axis] && split_position < bounds.max_p[axis]);

    if (left)
        bounds.max_p[axis] = split_position;
    else
        bounds.min_p[axis] = split_position;
}

template <typename Primitive_Source>
void KdTree_Builder<Primitive_Source>::build_node(const Bounding_Box& node_bounds, int32_t primitives_offset, int32_t primitive_count, int depth, int32_t above_primitives_offset)
{
    if (nodes.size() >= KdNode::max_node_count)
        error("maximum number of KdTree nodes has been reached: " + std::to_string(KdNode::max_node_count));

    // check if leaf node should be created
    if (primitive_count <= build_params.leaf_primitive_limit || depth == 0) {
        create_leaf(&primitive_buffer[primitives_offset], primitive_count);
        return;
    }

    // select split position
    Split split = select_split(node_bounds, &primitive_buffer[primitives_offset], primitive_count);
    if (split.edge == -1) {
        create_leaf(&primitive_buffer[primitives_offset], primitive_count);
        return;
    }
    float split_position = edges[split.axis][split.edge].position_on_axis;

    memcpy(primitive_buffer2.data(), &primitive_buffer[primitives_offset], primitive_count * sizeof(Primitive_Info));

    if (primitive_buffer.size() < above_primitives_offset + primitive_count)
        primitive_buffer.resize(primitive_buffer.size() + primitive_source.get_primitive_count());

    // classify primitives with respect to split
    int32_t n0 = 0;
    for (int32_t i = 0; i < split.edge; i++) {
        const Edge edge = edges[split.axis][i];
        if (edge.is_start() || edge.position_on_axis == split_position && edge.is_primitive_perpendicular_to_axis()) {
            int32_t index = edge.get_primitive_index();
            Primitive_Info primitive_info = primitive_buffer2[index];

            if (build_params.split_clipping) {
                if (primitive_info.bounds.max_p[split.axis] > split_position) {
                    clip_bounds(primitive_source, primitive_info.primitive, split_position, split.axis, true, primitive_info.bounds);
                }
            }

            primitive_buffer[n0++] = primitive_info;
        }
    }

    int32_t n1 = 0;
    for (int32_t i = split.edge + 1; i < 2 * primitive_count; i++) {
        const Edge edge = edges[split.axis][i];
        if (edge.is_end() || edge.position_on_axis == split_position && edge.is_primitive_perpendicular_to_axis()) {
            int32_t index = edge.get_primitive_index();
            Primitive_Info primitive_info = primitive_buffer2[index];

            if (build_params.split_clipping) {
                if (primitive_info.bounds.min_p[split.axis] < split_position) {
                    clip_bounds(primitive_source, primitive_info.primitive, split_position, split.axis, false, primitive_info.bounds);
                }
            }

            primitive_buffer[above_primitives_offset + n1++] = primitive_info;
        }
    }

    // add interior node and recursively create children nodes
    int32_t this_node_index = static_cast<int32_t>(nodes.size());
    nodes.push_back(KdNode());

    Bounding_Box bounds0 = node_bounds;
    bounds0.max_p[split.axis] = split_position;
    build_node(bounds0, 0, n0, depth - 1, above_primitives_offset + n1);

    int32_t above_child = static_cast<int32_t>(nodes.size());
    nodes[this_node_index].init_interior_node(split.axis, above_child, split_position);

    Bounding_Box bounds1 = node_bounds;
    bounds1.min_p[split.axis] = split_position;
    build_node(bounds1, above_primitives_offset, n1, depth - 1, above_primitives_offset);
}

template <typename Primitive_Source>
void KdTree_Builder<Primitive_Source>::create_leaf(const Primitive_Info* primitives, int32_t primitive_count)
{
    KdNode node;
    if (primitive_count == 0) {
        node.init_empty_leaf();
    }
    else if (primitive_count == 1) {
        node.init_leaf_with_single_primitive(primitives[0].primitive);
    }
    else {
        node.init_leaf_with_multiple_primitives(primitive_count, static_cast<int32_t>(primitive_indices.size()));
        for (int32_t i = 0; i < primitive_count; i++) {
            primitive_indices.push_back(primitives[i].primitive);
        }
    }
    nodes.push_back(node);
}

template <typename Primitive_Source>
Split KdTree_Builder<Primitive_Source>::select_split(const Bounding_Box& node_bounds, const Primitive_Info* primitives, int32_t primitive_count)
{
    // Determine axes iteration order.
    int axes[3];
    if (build_params.split_along_the_longest_axis) {
        Vector3 diag = node_bounds.max_p - node_bounds.min_p;
        if (diag.x >= diag.y && diag.x >= diag.z) {
            axes[0] = 0;
            axes[1] = diag.y >= diag.z ? 1 : 2;
        }
        else if (diag.y >= diag.x && diag.y >= diag.z) {
            axes[0] = 1;
            axes[1] = diag.x >= diag.z ? 0 : 2;
        }
        else {
            axes[0] = 2;
            axes[1] = diag.x >= diag.y ? 0 : 1;
        }
        axes[2] = 3 - axes[0] - axes[1]; // since 0 + 1 + 2 == 3
    }
    else {
        axes[0] = 0;
        axes[1] = 1;
        axes[2] = 2;
    }

    // Select spliting axis and position. If buildParams.splitAlongTheLongestAxis
    // is true then we stop at the first axis that gives a valid split.
    Split best_split = { -1, -1, std::numeric_limits<float>::infinity() };

    for (int axis : axes) {
        // initialize edges
        for (int32_t i = 0; i < primitive_count; i++) {
            const Bounding_Box& bounds = primitives[i].bounds;
            edges[axis][2 * i + 0] = { bounds.min_p[axis], static_cast<uint32_t>(i) };
            edges[axis][2 * i + 1] = { bounds.max_p[axis], static_cast<uint32_t>(i) | Edge::edge_end_flag };

            if (bounds.min_p[axis] == bounds.max_p[axis]) {
                edges[axis][2 * i + 0].primitive_and_flags |= Edge::primitive_perpendicular_to_axis_flag;
                edges[axis][2 * i + 1].primitive_and_flags |= Edge::primitive_perpendicular_to_axis_flag;
            }
        }

        std::stable_sort(edges[axis].data(), edges[axis].data() + 2 * primitive_count, Edge::less);

        // select split position
        Split split = select_split_for_axis(node_bounds, primitive_count, axis);
        if (split.edge != -1) {
            if (build_params.split_along_the_longest_axis)
                return split;
            if (split.cost < best_split.cost)
                best_split = split;
        }
    }

    return best_split;
}

template <typename Primitive_Source>
Split KdTree_Builder<Primitive_Source>::select_split_for_axis(const Bounding_Box& node_bounds, int32_t primitive_count, int axis) const
{
    static const int other_axis[3][2] = { {1, 2}, {0, 2}, {0, 1} };
    const int other_axis0 = other_axis[axis][0];
    const int other_axis1 = other_axis[axis][1];
    const Vector3 diag = node_bounds.max_p - node_bounds.min_p;

    const float s0 = 2.0f * (diag[other_axis0] * diag[other_axis1]);
    const float d0 = 2.0f * (diag[other_axis0] + diag[other_axis1]);

    const float inv_total_s = 1.0f / (2.0f * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z));

    const int32_t num_edges = 2 * primitive_count;

    Split best_split = { -1, axis, build_params.intersection_cost * primitive_count };

    int32_t num_below = 0;
    int32_t num_above = primitive_count;

    int32_t i = 0;
    while (i < num_edges) {
        Edge edge = edges[axis][i];

        // find group of edges with the same axis position: [i, group_end)
        int group_end = i + 1;
        while (group_end < num_edges && edge.position_on_axis == edges[axis][group_end].position_on_axis)
            group_end++;

        // [i, middle_edge) - edges End points.
        // [middle_edge, group_end) - edges Start points.
        int middle_edge = i;
        while (middle_edge != group_end && edges[axis][middle_edge].is_end())
            middle_edge++;

        num_above -= middle_edge - i;

        float t = edge.position_on_axis;
        if (t > node_bounds.min_p[axis] && t < node_bounds.max_p[axis]) {
            float below_s = s0 + d0 * (t - node_bounds.min_p[axis]);
            float above_s = s0 + d0 * (node_bounds.max_p[axis] - t);

            float p_below = below_s * inv_total_s;
            float p_above = above_s * inv_total_s;

            float empty_bonus = (num_below == 0 || num_above == 0) ? build_params.empty_bonus : 0.0f;

            float cost = build_params.traversal_cost + (1.0f - empty_bonus) * build_params.intersection_cost * (p_below * num_below + p_above * num_above);

            if (cost < best_split.cost) {
                best_split.edge = (middle_edge == group_end) ? middle_edge - 1 : middle_edge;
                best_split.cost = cost;
            }
        }

        num_below += group_end - middle_edge;
        i = group_end;
    }
    return best_split;
}
