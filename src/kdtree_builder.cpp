#include "common.h"
#include "kdtree_builder.h"
#include "triangle_mesh.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

struct Edge {
    float position_on_axis;
    uint32_t triangle_and_flag;

    enum : uint32_t { is_end_mask = 0x80000000 };
    enum : uint32_t { triangle_mask = 0x7fffffff };

    bool is_start() const {
        return (triangle_and_flag & is_end_mask) == 0;
    }

    bool is_end() const {
        return !is_start();
    }

    int32_t get_triangle_index() const {
        return static_cast<int32_t>(triangle_and_flag & triangle_mask);
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

struct Triangle_Info {
	int32_t triangle;
	Bounding_Box bounds;
};

class KdTree_Builder {
public:
    KdTree_Builder(const Triangle_Mesh& mesh, const KdTree_Build_Params& build_params);
    KdTree build();

private:
    void build_node(const Bounding_Box& node_bounds, int32_t triangles_offset, int32_t triangle_count,
        int depth, int32_t above_triangles_offset);

    void create_leaf(const Triangle_Info* triangles, int32_t triangle_count);
    Split select_split(const Bounding_Box& node_bounds, const Triangle_Info* triangles, int32_t triangle_count);
    Split select_split_for_axis(const Bounding_Box& node_bounds, int32_t triangle_count, int axis) const;

private:
    const Triangle_Mesh& mesh;
    KdTree_Build_Params build_params;

    std::vector<Edge> edges[3]; // edges for each axis
    std::vector<Triangle_Info> triangle_buffer;
    std::vector<Triangle_Info> triangle_buffer2;

    std::vector<KdNode> nodes;
    std::vector<int32_t> triangle_indices;
};

KdTree build_kdtree(const Triangle_Mesh& mesh, const KdTree_Build_Params& build_params) {
    KdTree_Builder builder(mesh, build_params);
    return builder.build();
}

enum {
  // max count is chosen such that maxTrianglesCount * 2 is still an int32_t,
  // this simplifies implementation.
  max_triangle_count = 0x3fffffff // max ~ 1 billion triangles
};

KdTree_Builder::KdTree_Builder(const Triangle_Mesh& mesh, const KdTree_Build_Params& build_params)
: mesh(mesh)
, build_params(build_params)
{
    if (mesh.get_triangle_count() > max_triangle_count) {
        error("exceeded the maximum number of mesh triangles: " + std::to_string(max_triangle_count));
    }

    if (this->build_params.max_depth <= 0) {
        this->build_params.max_depth = std::lround(8.0 + 1.3 * std::floor(std::log2(mesh.get_triangle_count())));
    }
    this->build_params.max_depth = std::min(this->build_params.max_depth, static_cast<int>(KdTree::max_traversal_depth));
}

KdTree KdTree_Builder::build()
{
    const int32_t triangle_count = mesh.get_triangle_count();

    //
    // Prepare working structures.
    //
    for (int i = 0; i < 3; i++)
        edges[i].resize(2 * triangle_count);

    // Maximum theoretical size of triangle_buffer is [triangle_count*(build_params.max_depth + 1)] elements.
    // Algorithm starts with the following initial size and resizes later if necessary.
    triangle_buffer.resize(static_cast<size_t>(triangle_count * 2.5));

    triangle_buffer2.resize(triangle_count);

    Bounding_Box mesh_bounds;
    for (int32_t i = 0; i < triangle_count; i++) {
        Bounding_Box bounds = mesh.get_triangle_bounds(i);
        triangle_buffer[i] = { i, bounds };
        mesh_bounds = Bounding_Box::get_union(mesh_bounds, bounds);
    }

    //
    // Recursively build all nodes.
    //
    build_node(mesh_bounds, 0, triangle_count, build_params.max_depth, triangle_count);

    return KdTree(std::move(nodes), std::move(triangle_indices), mesh);
}

//
// Splits the provided bounding box and selects either left or right part based on the provided boolean flag.
// The selected part is additionally clipped to be as tight as possible taking into account triangle geometry.
// Implements clipping algorithm described in this article:
//      Alexei Soupikov, Maxim Shevtsov, Alexander Kapustin, 2008. Improving Kd-tree Quality at a Reasonable Construction Cost
//
void clip_bounds(const Triangle& tri, float split_position, int axis, bool left, Bounding_Box& bounds)
{
    assert(split_position > bounds.min_p[axis] && split_position < bounds.max_p[axis]);

    if (left)
        bounds.max_p[axis] = split_position;
    else
        bounds.min_p[axis] = split_position;

    // sort triangle vertices along the split dimension
    Vector p[3] = { tri[0], tri[1], tri[2] };

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

    Vector a, c;
    Vector b = p[1];
    if (middle_on_the_left) {
        a = p[2];
        c = p[0];
    } else {
        a = p[0];
        c = p[2];
    }

    // find insersection points of two edges with a splitting plane
    Vector isect_ab;
    if (b[axis] == split_position) {
        isect_ab = b;
    } else {
        Vector ab = b - a;
        isect_ab = a + ab * ((split_position - a[axis]) / ab[axis]);
    }

    Vector ac = c - a;
    Vector isect_ac = a + ac * ((split_position - a[axis]) / ac[axis]);

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
    bounds = Bounding_Box::get_intersection(bounds, bounds2);
}

void KdTree_Builder::build_node(const Bounding_Box& node_bounds, int32_t triangles_offset, int32_t triangle_count, int depth, int32_t above_triangles_offset)
{
    if (nodes.size() >= KdNode::max_node_count)
        error("maximum number of KdTree nodes has been reached: " + std::to_string(KdNode::max_node_count));

    // check if leaf node should be created
    if (triangle_count <= build_params.leaf_triangles_limit || depth == 0) {
        create_leaf(&triangle_buffer[triangles_offset], triangle_count);
        return;
    }

    // select split position
    auto split = select_split(node_bounds, &triangle_buffer[triangles_offset], triangle_count);
    if (split.edge == -1) {
        create_leaf(&triangle_buffer[triangles_offset], triangle_count);
        return;
    }
    float split_position = edges[split.axis][split.edge].position_on_axis;

    memcpy(triangle_buffer2.data(), &triangle_buffer[triangles_offset], triangle_count * sizeof(Triangle_Info));

    if (triangle_buffer.size() < above_triangles_offset + triangle_count)
        triangle_buffer.resize(triangle_buffer.size() + mesh.get_triangle_count());

    // classify triangles with respect to split
    int32_t n0 = 0;
    for (int32_t i = 0; i < split.edge; i++) {
        if (edges[split.axis][i].is_start()) {
            int32_t index = edges[split.axis][i].get_triangle_index();
            Triangle_Info triangle_info = triangle_buffer2[index];

            if (build_params.split_clipping) {
                if (triangle_info.bounds.max_p[split.axis] > split_position) {
                    const Triangle& triangle = mesh.get_triangle(triangle_info.triangle);
                    clip_bounds(triangle, split_position, split.axis, true, triangle_info.bounds);
                }
            }

            triangle_buffer[n0++] = triangle_info;
        }
    }

    int32_t n1 = 0;
    for (int32_t i = split.edge + 1; i < 2 * triangle_count; i++) {
        if (edges[split.axis][i].is_end()) {
            int32_t index = edges[split.axis][i].get_triangle_index();
            Triangle_Info triangle_info = triangle_buffer2[index];

            if (build_params.split_clipping) {
                if (triangle_info.bounds.min_p[split.axis] < split_position) {
                    const Triangle& triangle = mesh.get_triangle(triangle_info.triangle);
                    clip_bounds(triangle, split_position, split.axis, false, triangle_info.bounds);
                }
            }

            triangle_buffer[above_triangles_offset + n1++] = triangle_info;
        }
    }

    // add interior node and recursively create children nodes
    auto this_node_index = static_cast<int32_t>(nodes.size());
    nodes.push_back(KdNode());

    Bounding_Box bounds0 = node_bounds;
    bounds0.max_p[split.axis] = split_position;
    build_node(bounds0, 0, n0, depth - 1, above_triangles_offset + n1);

    auto above_child = static_cast<int32_t>(nodes.size());
    nodes[this_node_index].init_interior_node(split.axis, above_child, split_position);

    Bounding_Box bounds1 = node_bounds;
    bounds1.min_p[split.axis] = split_position;
    build_node(bounds1, above_triangles_offset, n1, depth - 1, above_triangles_offset);
}

void KdTree_Builder::create_leaf(const Triangle_Info* triangles, int32_t triangle_count)
{
    KdNode node;
    if (triangle_count == 0) {
        node.init_empty_leaf();
    }
    else if (triangle_count == 1) {
        node.init_leaf_with_single_triangle(triangles[0].triangle);
    }
    else {
        node.init_leaf_with_multiple_triangles(triangle_count, static_cast<int32_t>(triangle_indices.size()));
        for (int32_t i = 0; i < triangle_count; i++) {
            triangle_indices.push_back(triangles[i].triangle);
        }
    }
    nodes.push_back(node);
}

Split KdTree_Builder::select_split(const Bounding_Box& node_bounds, const Triangle_Info* triangles, int32_t triangle_count)
{
    // Determine axes iteration order.
    int axes[3];
    if (build_params.split_along_the_longest_axis) {
        Vector diag = node_bounds.max_p - node_bounds.min_p;
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
        for (int32_t i = 0; i < triangle_count; i++) {
            auto& bounds = triangles[i].bounds;
            edges[axis][2 * i + 0] = { bounds.min_p[axis], static_cast<uint32_t>(i) };
            edges[axis][2 * i + 1] = { bounds.max_p[axis], static_cast<uint32_t>(i) | Edge::is_end_mask };
        }

        std::stable_sort(edges[axis].data(), edges[axis].data() + 2 * triangle_count, Edge::less);

        // select split position
        auto split = select_split_for_axis(node_bounds, triangle_count, axis);
        if (split.edge != -1) {
            if (build_params.split_along_the_longest_axis)
                return split;
            if (split.cost < best_split.cost)
                best_split = split;
        }
    }

    return best_split;
}

Split KdTree_Builder::select_split_for_axis(const Bounding_Box& node_bounds, int32_t triangle_count, int axis) const
{
    static const int other_axis[3][2] = { {1, 2}, {0, 2}, {0, 1} };
    const int other_axis0 = other_axis[axis][0];
    const int other_axis1 = other_axis[axis][1];
    const Vector diag = node_bounds.max_p - node_bounds.min_p;

    const float s0 = 2.0f * (diag[other_axis0] * diag[other_axis1]);
    const float d0 = 2.0f * (diag[other_axis0] + diag[other_axis1]);

    const float inv_total_s = 1.0f / (2.0f * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z));

    const int32_t num_edges = 2 * triangle_count;

    Split best_split = { -1, axis, build_params.intersection_cost * triangle_count };

    int32_t num_below = 0;
    int32_t num_above = triangle_count;

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

            auto empty_bonus = (num_below == 0 || num_above == 0) ? build_params.empty_bonus : 0.0f;

            auto cost = build_params.traversal_cost + (1.0f - empty_bonus) * build_params.intersection_cost * (p_below * num_below + p_above * num_above);

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
