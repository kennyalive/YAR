#include "std.h"
#include "lib/common.h"
#include "kdtree.h"

#include "intersection.h"

//#define BRUTE_FORCE_INTERSECTION

template <typename Primitive_Source>
KdTree<Primitive_Source>::KdTree(std::vector<KdNode>&& nodes, std::vector<int32_t>&& primitive_indices, Primitive_Source&& primitive_source)
: nodes(std::move(nodes))
, primitive_indices(std::move(primitive_indices))
, primitive_source(std::move(primitive_source))
{
    bounds = this->primitive_source.calculate_bounds();
}

template <typename Primitive_Source>
bool KdTree<Primitive_Source>::intersect_any(const Ray& ray, float tmax) const {
    Intersection intersection{ tmax };
    return intersect(ray, intersection);
}

template <typename Primitive_Source>
bool KdTree<Primitive_Source>::intersect(const Ray& ray, Intersection& intersection) const {
#ifdef BRUTE_FORCE_INTERSECTION
    return intersect_brute_force(ray, intersection);
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
            int axis = node->get_split_axis();
            float distance_to_split_plane = node->get_split_position() - ray.origin[axis];

            const KdNode* below_child = node + 1;
            const KdNode* above_child = &nodes[node->get_above_child()];

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
                if (ray.direction[axis] > 0.0) {
                    if (t_min > 0.0)
                        node = above_child;
                    else { // t_min == 0.0
                        ASSERT(traversal_stack_size < max_traversal_depth);
                        traversal_stack[traversal_stack_size++] = {above_child, 0.0, t_max};
                        // check single point [0.0, 0.0]
                        node = below_child;
                        t_max = 0.0;
                    }
                }
                else if (ray.direction[axis] < 0.0) {
                    if (t_min > 0.0)
                        node = below_child;
                    else { // t_min == 0.0
                        ASSERT(traversal_stack_size < max_traversal_depth);
                        traversal_stack[traversal_stack_size++] = {below_child, 0.0, t_max};
                        // check single point [0.0, 0.0]
                        node = above_child;
                        t_max = 0.0;
                    }
                }
                else { // ray.direction[axis] == 0.0
                    // for both nodes check [t_min, t_max] range
                    ASSERT(traversal_stack_size < max_traversal_depth);
                    traversal_stack[traversal_stack_size++] = {above_child, t_min, t_max};
                    node = below_child;
                }
            }
        }
        else { // leaf node
            intersect_leaf(ray, *node, intersection);

            if (traversal_stack_size == 0)
                break;

            // Almost correct implementation is just: --traversal_stack_size.
            // We need to scan the entire stack to handle the case when distance_to_split_plane == 0.0 && ray.direction[axis] == 0.0.
            do {
                --traversal_stack_size;
            } while (traversal_stack_size > 0 && traversal_stack[traversal_stack_size].t_min >= intersection.t);

            node = traversal_stack[traversal_stack_size].node;
            t_min = traversal_stack[traversal_stack_size].t_min;
            t_max = traversal_stack[traversal_stack_size].t_max;
        }
    } // while (intersection.t > t_min)
    return intersection.t < ray_tmax;
#endif // !BRUTE_FORCE_INTERSECTION
}

template <typename Primitive_Source>
void KdTree<Primitive_Source>::intersect_leaf(const Ray& ray, KdNode leaf, Intersection& intersection) const
{
    if (leaf.get_primitive_count() == 1) {
        primitive_source.intersect_primitive(ray, leaf.get_index(), intersection);
        return;
    }

    for (int32_t i = 0; i < leaf.get_primitive_count(); i++) {
        int32_t primitive_index = primitive_indices[leaf.get_index() + i];
        primitive_source.intersect_primitive(ray, primitive_index, intersection);
    }
}

template <typename Primitive_Source>
bool KdTree<Primitive_Source>::intersect_brute_force(const Ray& ray, Intersection& intersection) const {
    float tmax = intersection.t;
    for (int i = 0; i < primitive_source.get_primitive_count(); i++) {
        primitive_source.intersect_primitive(ray, i, intersection);
    }
    return intersection.t < tmax;
}

template <typename Primitive_Source>
KdTree_Stats KdTree<Primitive_Source>::calculate_stats() const
{
    KdTree_Stats stats;

    stats.nodes_size = nodes.size() * sizeof(KdNode);
    stats.primitive_indices_size = primitive_indices.size() * sizeof(primitive_indices[0]);
    stats.node_count = static_cast<int32_t>(nodes.size());

    int64_t primitive_per_leaf_accumulated = 0;

    for (auto node : nodes) {
        if (node.is_leaf()) {
            stats.leaf_count++;
            primitive_per_leaf_accumulated += node.get_primitive_count();

            if (node.get_primitive_count() == 0)
                stats.empty_leaf_count++;
            else if (node.get_primitive_count() == 1)
                stats.single_primitive_leaf_count++;
        }
    }

    auto not_empty_leaf_count = stats.leaf_count - stats.empty_leaf_count;

    stats.perfect_depth = static_cast<int>(std::ceil(std::log2(stats.leaf_count)));
    stats.not_empty_leaf_stats.average_primitive_count = float(double(primitive_per_leaf_accumulated) / not_empty_leaf_count);

    // Compute depth of each leaf node.
    std::vector<uint8_t> not_empty_leaf_depth_values;
    std::vector<uint8_t> empty_leaf_depth_values;

    struct Depth_Info {
        int32_t node_index = -1;
        uint8_t depth = -1;
    };
    std::vector<Depth_Info> depth_info{ Depth_Info{0, 0} };

    size_t i = 0;
    while (i < depth_info.size()) {
        int32_t node_index = depth_info[i].node_index;
        uint8_t depth = depth_info[i].depth;

        if (nodes[node_index].is_leaf()) {
            if (nodes[node_index].get_primitive_count() > 0)
                not_empty_leaf_depth_values.push_back(depth);
            else
                empty_leaf_depth_values.push_back(depth);
        }
        else {
            int32_t below_child_index = node_index + 1;
            depth_info.push_back({ below_child_index, uint8_t(depth + 1) });

            int32_t above_child_index = nodes[node_index].get_above_child();
            depth_info.push_back({ above_child_index, uint8_t(depth + 1) });
        }
        i++;
    }

    int64_t not_empty_leaf_depth_accumulated = std::accumulate(not_empty_leaf_depth_values.cbegin(), not_empty_leaf_depth_values.cend(), int64_t(0));
    stats.not_empty_leaf_stats.average_depth = float(double(not_empty_leaf_depth_accumulated) / not_empty_leaf_count);

    double accum = 0.0;
    for (auto depth : not_empty_leaf_depth_values) {
        auto diff = depth - stats.not_empty_leaf_stats.average_depth;
        accum += diff * diff;
    }
    stats.not_empty_leaf_stats.depth_standard_deviation = float(std::sqrt(accum / not_empty_leaf_count));

    int64_t empty_leaf_depth_accumulated = std::accumulate(empty_leaf_depth_values.cbegin(), empty_leaf_depth_values.cend(), int64_t(0));
    stats.empty_leaf_stats.average_depth = float(double(empty_leaf_depth_accumulated) / stats.empty_leaf_count);

    accum = 0.0f;
    for (auto depth : empty_leaf_depth_values) {
        auto diff = depth - stats.empty_leaf_stats.average_depth;
        accum += diff * diff;
    }
    stats.empty_leaf_stats.depth_standard_deviation = float(std::sqrt(accum / stats.empty_leaf_count));

    return stats;
}

template <typename Primitive_Source>
std::vector<int32_t> KdTree<Primitive_Source>::calculate_path_to_node(int32_t node_index) const
{
    ASSERT(node_index >= 0 && node_index < nodes.size());

    std::map<int32_t, int32_t> parent_map;

    for (int32_t i = 0; i < int32_t(nodes.size()); i++) {
        auto node = nodes[i];
        if (!node.is_leaf()) {
            int32_t below_child = i + 1;
            int32_t above_child = node.get_above_child();
            parent_map[below_child] = i;
            parent_map[above_child] = i;
        }
    }

    std::vector<int32_t> path { node_index };
    auto it = parent_map.find(node_index);
    while (it != parent_map.cend()) {
        path.push_back(it->second);
        it = parent_map.find(it->second);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void KdTree_Stats::print()
{
    printf("[memory consumption]\n");
    printf("\tnodes_size = %zdK\n", nodes_size / 1024);
    printf("\tprimitive_indices_size = %zdK\n", primitive_indices_size / 1024);

    printf("[general]\n");
    printf("\tnode_count = %d\n", node_count);
    printf("\tleaf_count = %d\n", leaf_count);
    printf("\tempty_leaf_count = %d\n", empty_leaf_count);
    printf("\tsingle_primitive_leaf_count = %d\n", single_primitive_leaf_count);
    printf("\tperfect_depth = %d\n", perfect_depth);

    printf("[non-empty leaves]\n");
    printf("\taverage_depth = %.2f\n", not_empty_leaf_stats.average_depth);
    printf("\tdepth_standard_deviation = %.2f\n", not_empty_leaf_stats.depth_standard_deviation);
    printf("\taverage_primitive_count = %.2f\n", not_empty_leaf_stats.average_primitive_count);

    printf("[empty leaves]\n");
    printf("\taverage_depth = %.2f\n", empty_leaf_stats.average_depth);
    printf("\tdepth_standard_deviation = %.2f\n", empty_leaf_stats.depth_standard_deviation);
}

template <typename Primitive_Source>
void KdTree<Primitive_Source>::save_to_file(const std::string& file_name) const
{
    std::ofstream file(file_name, std::ios_base::out | std::ios_base::binary);
    if (!file)
        error("failed to open kdTree file for writing: " + file_name);

    // write nodes
    int32_t node_count = static_cast<int32_t>(nodes.size());
    file.write(reinterpret_cast<const char*>(&node_count), 4);

    size_t nodes_byte_count = node_count * sizeof(KdNode);
    file.write(reinterpret_cast<const char*>(nodes.data()), nodes_byte_count);
    if (!file)
        error("failed to write kdTree nodes: " + file_name);

    // write primitive indices
    int32_t index_count = static_cast<int32_t>(primitive_indices.size());
    file.write(reinterpret_cast<const char*>(&index_count), 4);

    size_t indices_byte_count = index_count * 4;
    file.write(reinterpret_cast<const char*>(primitive_indices.data()),
        indices_byte_count);
    if (!file)
        error("failed to write kdTree primitive indices: " + file_name);
}

Geometry_KdTree load_geometry_kdtree(const std::string& file_name, const Geometries* geometries, Geometry_Handle geometry) {
    Geometry_KdTree kdtree;

    kdtree.primitive_source = Geometry_Primitive_Source(geometries, geometry);
    kdtree.bounds = kdtree.primitive_source.calculate_bounds();

    std::ifstream file(file_name, std::ios_base::in | std::ios_base::binary);
    if (!file)
        error("failed to open kdTree file: " + file_name);

    // read nodes
    int32_t node_count;
    file.read(reinterpret_cast<char*>(&node_count), 4);
    if (!file)
        error("failed to read nodes count: " + file_name);

    auto& mutable_nodes = const_cast<std::vector<KdNode>&>(kdtree.nodes);
    mutable_nodes.resize(node_count);

    size_t nodes_byte_count = node_count * sizeof(KdNode);
    file.read(reinterpret_cast<char*>(mutable_nodes.data()), nodes_byte_count);
    if (!file)
        error("failed to read kdTree nodes: " + file_name);

    // read primitive indices
    int32_t index_count;
    file.read(reinterpret_cast<char*>(&index_count), 4);
    if (!file)
        error("failed to read primitive indices count: " + file_name);

    auto& mutable_indices = const_cast<std::vector<int32_t>&>(kdtree.primitive_indices);
    mutable_indices.resize(index_count);

    size_t indices_byte_count = index_count * 4;
    file.read(reinterpret_cast<char*>(mutable_indices.data()), indices_byte_count);
    if (!file)
        error("failed to read kdTree primitive indices: " + file_name);

    return kdtree;
}

template class KdTree<Geometry_Primitive_Source>;
template class KdTree<Scene_Primitive_Source>;
