#include "std.h"
#include "lib/common.h"
#include "kdtree_builder.h"

#include "lib/scene_object.h"
#include "lib/triangle_mesh.h"

constexpr float empty_node_bonus = 0.3f;
constexpr int leaf_primitive_count_threshold = 2;

// Splits the bounding box and selects either left or right part based on the provided boolean flag.
// The selected part is additionally clipped to be as tight as possible taking into account triangle geometry.
// 
// Implements clipping algorithm described here:
//      Alexei Soupikov, Maxim Shevtsov, Alexander Kapustin, 2008. Improving Kd-tree Quality at a Reasonable Construction Cost
//
static void clip_bounds(const Triangle_Mesh& mesh, uint32_t triangle_index, float split_position, int axis, bool left, Bounding_Box& bounds)
{
    ASSERT(split_position > bounds.min_p[axis] && split_position < bounds.max_p[axis]);
    if (left)
        bounds.max_p[axis] = split_position;
    else
        bounds.min_p[axis] = split_position;

    // sort triangle vertices along the split dimension
    Vector3 p[3];
    mesh.get_triangle(triangle_index, p[0], p[1], p[2]);

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
    }
    else {
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
    }
    else {
        bounds2.add_point(p[2]);
        if (!middle_on_the_left)
            bounds2.add_point(p[1]);
    }
    bounds = Bounding_Box::compute_intersection(bounds, bounds2);
}

namespace {
    struct Edge {
        float position_on_axis;
        uint32_t primitive_and_flags;

        static constexpr uint32_t edge_end_flag = 0x80000000;
        static constexpr uint32_t primitive_perpendicular_to_axis_flag = 0x40000000;
        static constexpr uint32_t primitive_mask = 0x3fffffff;

        bool is_start() const {
            return (primitive_and_flags & edge_end_flag) == 0;
        }

        bool is_end() const {
            return !is_start();
        }

        bool is_primitive_perpendicular_to_axis() const {
            return (primitive_and_flags & primitive_perpendicular_to_axis_flag) != 0;
        }

        uint32_t get_primitive_index() const {
            return primitive_and_flags & primitive_mask;
        }

        static bool less(Edge edge1, Edge edge2) {
            if (edge1.position_on_axis == edge2.position_on_axis)
                return edge1.is_end() && edge2.is_start();
            else
                return edge1.position_on_axis < edge2.position_on_axis;
        }
    };

    struct Primitive_Info {
        uint32_t primitive;
        Bounding_Box bounds;
    };
} // namespace

struct KdTree_Builder {
    KdTree_Builder(
        uint32_t total_primitive_count,
        std::function<Bounding_Box (uint32_t)> get_primitive_bounds,  // get bounds for primitive with a given index
        const Triangle_Mesh* mesh // optional, used if build kdtree for triangle mesh geometry
    );

    void build()
    {
        build_node(total_bounds, 0, total_primitive_count, max_depth, total_primitive_count);
    }

    void build_node(const Bounding_Box& node_bounds, uint32_t primitives_offset, uint32_t primitive_count,
        int depth, uint32_t above_primitive_offset);

    void create_leaf(const Primitive_Info* primitives, uint32_t primitive_count);

    // Returns axis along which the split is made.
    // split_edge - the edge selected for split.
    int select_split(const Bounding_Box& node_bounds, const Primitive_Info* primitives, uint32_t primitive_count, uint32_t* split_edge);

    // Returns the value of the cost function for the selected split position.
    // Returns Infinity if split is not possible along the given axis.
    // split_edge - the edge selected for split.
    float select_split_for_axis(const Bounding_Box& node_bounds, uint32_t primitive_count, int axis, uint32_t* split_edge) const;

    const uint32_t total_primitive_count = 0;
    const Triangle_Mesh* mesh = nullptr; // not null if we build kdtree for triangle mesh
    const int max_depth = 0;

    Bounding_Box total_bounds;

    // Intermediate storage
    std::vector<Edge> edges[3]; // edges for each axis
    std::vector<Primitive_Info> primitive_buffer;
    std::vector<Primitive_Info> primitive_buffer2;

    // Created nodes
    std::vector<KdNode> nodes;
    std::vector<uint32_t> primitive_indices;
};

KdTree_Builder::KdTree_Builder(
    uint32_t total_primitive_count,
    std::function<Bounding_Box(uint32_t)> get_primitive_bounds,
    const Triangle_Mesh* mesh
)
    : total_primitive_count(total_primitive_count)
    , mesh(mesh)
    , max_depth(KdTree::get_max_depth_limit(total_primitive_count))
{
    // Edge::primitive_and_flags reserves 2 bits for flags and 30 bits are left for primitive index.
    static constexpr uint32_t max_primitive_count = 0x3fffffff; //  max ~ 1 billion primitives
    if (total_primitive_count > max_primitive_count) {
        error("exceeded the maximum number of primitives: " + std::to_string(max_primitive_count));
    }

    primitive_buffer.reserve(total_primitive_count);
    for (uint32_t i = 0; i < total_primitive_count; i++) {
        Bounding_Box bounds = get_primitive_bounds(i);
        primitive_buffer.push_back({ i, bounds});
        total_bounds = Bounding_Box::compute_union(total_bounds, bounds);
    }

    for (int i = 0; i < 3; i++)
        edges[i].resize(2 * total_primitive_count);

    // Maximum theoretical size is [total_primitive_count*(build_params.max_depth + 1)] elements.
    // Algorithm starts with the following initial size and resizes later if necessary.
    primitive_buffer.resize(size_t(total_primitive_count * 2.5));

    primitive_buffer2.resize(total_primitive_count);
}

void KdTree_Builder::build_node(const Bounding_Box& node_bounds, uint32_t primitives_offset, uint32_t primitive_count, int depth, uint32_t above_primitives_offset)
{
    if (nodes.size() >= KdNode::max_node_count)
        error("maximum number of KdTree nodes has been reached: " + std::to_string(KdNode::max_node_count));

    // check if leaf node should be created
    if (primitive_count <= (uint32_t)leaf_primitive_count_threshold || depth == 0) {
        create_leaf(&primitive_buffer[primitives_offset], primitive_count);
        return;
    }

    // select split position
    uint32_t split_edge;
    int split_axis = select_split(node_bounds, &primitive_buffer[primitives_offset], primitive_count, &split_edge);
    if (split_axis == -1) {
        create_leaf(&primitive_buffer[primitives_offset], primitive_count);
        return;
    }
    float split_position = edges[split_axis][split_edge].position_on_axis;

    memcpy(primitive_buffer2.data(), &primitive_buffer[primitives_offset], primitive_count * sizeof(Primitive_Info));

    if (primitive_buffer.size() < above_primitives_offset + primitive_count)
        primitive_buffer.resize(primitive_buffer.size() + total_primitive_count);

    // classify primitives with respect to split

    // NOTE ABOUT PRIMITIVES IN THE SPLITTING PLANE: Primitives that lie in the splitting plane require special
    // handling. Edge::less() comparator arranges edges with the same position by putting at first endpoints and
    // then startpoints. This works correctly for all primitives except the ones that lie in the splitting plane.
    // For them, endpoints will be on the left of the splitting plane and startpoints on the right - it means both
    // edges will be skipped by the code that classifies primitives with respect split (and the entire primitive
    // will be excluded from the tree). 
    //
    // The solution is to handle such primitives explicitly by checking that if a primitive is in the splitting
    // plane then add it to the left node, even if the edge is 'endpoint'. In order to not duplicate the same 
    // primitive in the left and in right node we add it only to the left node.
    //
    // There is one subtlety that should be taken into account to proof this code is correct. It's not immediately
    // obvious if it's correct to use 'endpoint' but not 'startpoint' in order to prevent duplication.
    // What if this specific 'endpoint' was selected as a 'splitting plane edge' - in that case it is not considered
    // by the classification code. It can be shown this can't happen. Splitting edge selection algorithm always
    // selects the first 'startpoint' if there are multiple 'endpoints' and 'startpoints' at the same position. And
    // for the primitive in the clipping plane we always have one 'endpoint' and then one 'startpoint', so 'startpoint'
    // will be used as a splitting edge, if necessary, and we have a guarantee that 'endpoint' will be part of classification.

    uint32_t n0 = 0;
    for (uint32_t i = 0; i < split_edge; i++) {
        const Edge edge = edges[split_axis][i];

        if (edge.is_start() ||
            (edge.position_on_axis == split_position && edge.is_primitive_perpendicular_to_axis()))
        {
            uint32_t index = edge.get_primitive_index();
            Primitive_Info primitive_info = primitive_buffer2[index];

            if (primitive_info.bounds.max_p[split_axis] > split_position) {
                if (mesh)
                    clip_bounds(*mesh, primitive_info.primitive, split_position, split_axis, true, primitive_info.bounds);
            }
            primitive_buffer[n0++] = primitive_info;
        }
    }
    ASSERT(n0 <= primitive_count);

    uint32_t n1 = 0;
    for (uint32_t i = split_edge + 1; i < 2 * primitive_count; i++) {
        const Edge edge = edges[split_axis][i];
        if (edge.is_end()) {
            uint32_t index = edge.get_primitive_index();
            Primitive_Info primitive_info = primitive_buffer2[index];

            if (primitive_info.bounds.min_p[split_axis] < split_position) {
                if (mesh)
                    clip_bounds(*mesh, primitive_info.primitive, split_position, split_axis, false, primitive_info.bounds);
            }
            primitive_buffer[above_primitives_offset + n1++] = primitive_info;
        }
    }
    ASSERT(n1 <= primitive_count);

    // add interior node and recursively create children nodes
    uint32_t this_node_index = (uint32_t)nodes.size();
    nodes.push_back(KdNode());

    Bounding_Box bounds0 = node_bounds;
    bounds0.max_p[split_axis] = split_position;
    build_node(bounds0, 0, n0, depth - 1, above_primitives_offset + n1);

    uint32_t above_child = (uint32_t)nodes.size();
    nodes[this_node_index].init_interior_node(split_axis, above_child, split_position);

    Bounding_Box bounds1 = node_bounds;
    bounds1.min_p[split_axis] = split_position;
    build_node(bounds1, above_primitives_offset, n1, depth - 1, above_primitives_offset);
}

void KdTree_Builder::create_leaf(const Primitive_Info* primitives, uint32_t primitive_count)
{
    KdNode node;
    if (primitive_count == 0) {
        node.init_empty_node();
    }
    else if (primitive_count == 1) {
        node.init_leaf_with_single_primitive(primitives[0].primitive);
    }
    else {
        node.init_leaf_with_multiple_primitives(primitive_count, (uint32_t)primitive_indices.size());
        for (uint32_t i = 0; i < primitive_count; i++) {
            primitive_indices.push_back(primitives[i].primitive);
        }
        // Keep leaf primitive indices ordered to simplify debugging/thinking.
        std::sort(primitive_indices.end() - primitive_count, primitive_indices.end());
    }
    nodes.push_back(node);
}

int KdTree_Builder::select_split(const Bounding_Box& node_bounds, const Primitive_Info* primitives, uint32_t primitive_count, uint32_t* split_edge)
{
    float best_cost = Infinity;
    int best_axis = -1;
    uint32_t best_edge;

    for (int axis : {0, 1, 2}) {
        // initialize edges
        for (uint32_t i = 0; i < primitive_count; i++) {
            const Bounding_Box& bounds = primitives[i].bounds;
            edges[axis][2 * i + 0] = { bounds.min_p[axis], (uint32_t)i };
            edges[axis][2 * i + 1] = { bounds.max_p[axis], (uint32_t)i | Edge::edge_end_flag };

            if (bounds.min_p[axis] == bounds.max_p[axis]) {
                edges[axis][2 * i + 0].primitive_and_flags |= Edge::primitive_perpendicular_to_axis_flag;
                edges[axis][2 * i + 1].primitive_and_flags |= Edge::primitive_perpendicular_to_axis_flag;
            }
        }
        std::stable_sort(edges[axis].data(), edges[axis].data() + 2 * primitive_count, Edge::less);

        // select split position
        uint32_t edge;
        float cost = select_split_for_axis(node_bounds, primitive_count, axis, &edge);
        if (cost < best_cost) {
            best_cost = cost;
            best_axis = axis;
            best_edge = edge;
        }
    }

    if (best_axis != -1) {
        *split_edge = best_edge;
    }
    return best_axis;
}

float KdTree_Builder::select_split_for_axis(const Bounding_Box& node_bounds, uint32_t primitive_count, int axis, uint32_t* split_edge) const
{
    static constexpr int other_axis[3][2] = { {1, 2}, {0, 2}, {0, 1} };
    const int other_axis0 = other_axis[axis][0];
    const int other_axis1 = other_axis[axis][1];

    const Vector3 diag = node_bounds.max_p - node_bounds.min_p;
    const float s0 = 2.0f * (diag[other_axis0] * diag[other_axis1]);
    const float d0 = 2.0f * (diag[other_axis0] + diag[other_axis1]);
    const float inv_total_area = 1.0f / (2.0f * (diag.x * diag.y + diag.x * diag.z + diag.y * diag.z));

    const uint32_t num_edges = 2 * primitive_count;

    float best_cost = (float)primitive_count;
    uint32_t best_edge = (uint32_t)-1;

    uint32_t num_below = 0;
    uint32_t num_above = primitive_count;

    uint32_t i = 0;
    while (i < num_edges) {
        const float t = edges[axis][i].position_on_axis;

        // find group of edges with the same axis position: [i, group_end)
        uint32_t group_end = i + 1;
        while (group_end < num_edges && t == edges[axis][group_end].position_on_axis)
            group_end++;

        // [i, middle_edge) - edges End points.
        // [middle_edge, group_end) - edges Start points.
        uint32_t middle_edge = i;
        while (middle_edge != group_end && edges[axis][middle_edge].is_end())
            middle_edge++;

        num_above -= middle_edge - i;
        if (t > node_bounds.min_p[axis] && t < node_bounds.max_p[axis]) {
            float below_area = s0 + d0 * (t - node_bounds.min_p[axis]);
            float above_area = s0 + d0 * (node_bounds.max_p[axis] - t);

            float p_below = below_area * inv_total_area;
            float p_above = above_area * inv_total_area;

            float empty_bonus_value = (num_below == 0 || num_above == 0) ? empty_node_bonus : 0.0f;
            float intersection_count_expected_value = p_below * num_below + p_above * num_above;
            float cost = (1.f - empty_bonus_value) * intersection_count_expected_value;

            if (cost < best_cost) {
                best_edge = (middle_edge == group_end) ? middle_edge - 1 : middle_edge;
                best_cost = cost;
            }
        }
        num_below += group_end - middle_edge;
        i = group_end;
    }
    ASSERT(num_below == primitive_count);
    ASSERT(num_above == 0);

    if (best_edge == (uint32_t)-1) {
        return Infinity;
    }
    else {
        *split_edge = best_edge;
        return best_cost;
    }
}

KdTree build_triangle_mesh_kdtree(const Triangle_Mesh_Geometry_Data* triangle_mesh_geometry_data)
{
    const Triangle_Mesh* mesh = triangle_mesh_geometry_data->mesh;
    auto get_primitive_bounds = [mesh](uint32_t index) {
        return mesh->get_triangle_bounds(index);
    };

    KdTree_Builder builder(mesh->get_triangle_count(), get_primitive_bounds, mesh);
    builder.build();

    KdTree tree;
    tree.bounds = builder.total_bounds;
    tree.geometry_data_hash = KdTree::compute_triangle_mesh_hash(*mesh);
    tree.nodes = std::move(builder.nodes);
    tree.primitive_indices = std::move(builder.primitive_indices);
    tree.set_geometry_data(triangle_mesh_geometry_data);

#if USE_KD_TILES
    if (!tree.nodes[0].is_leaf()) {
        std::vector<uint8_t> tiles = convert_kdtree_nodes_to_tiled_layout(tree);
        tree.tile_buffer = std::unique_ptr<uint8_t, KdTree::Tile_Buffer_Deleter>(
            (uint8_t*)_aligned_malloc(tiles.size(), cache_line_size)
            );
        memcpy(tree.tile_buffer.get(), tiles.data(), tiles.size());
        tree.tile_buffer_size = tiles.size();
    }
#endif

    return tree;
}

KdTree build_scene_kdtree(const Scene_Geometry_Data* scene_geometry_data)
{
    auto get_primitive_bounds = [scene_geometry_data](uint32_t index) {
        const Scene_Object& object = (*scene_geometry_data->scene_objects)[index];
        int offset = scene_geometry_data->geometry_type_offsets[static_cast<int>(object.geometry.type)];
        int kdtree_index = offset + object.geometry.index;
        Bounding_Box local_bounds = (*scene_geometry_data->kdtrees)[kdtree_index].bounds;
        Bounding_Box world_bounds = transform_bounding_box(object.object_to_world_transform, local_bounds);
        return world_bounds;
    };

    KdTree_Builder builder((uint32_t)scene_geometry_data->scene_objects->size(), get_primitive_bounds, nullptr);
    builder.build();

    KdTree tree;
    tree.bounds = builder.total_bounds;
    tree.geometry_data_hash = KdTree::compute_scene_kdtree_data_hash(*scene_geometry_data);
    tree.nodes = std::move(builder.nodes);
    tree.primitive_indices = std::move(builder.primitive_indices);
    tree.set_geometry_data(scene_geometry_data);
    return tree;
}
