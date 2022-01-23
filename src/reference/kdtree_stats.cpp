#include "std.h"
#include "lib/common.h"

#include "kdtree_stats.h"
#include "kdtree.h"

KdTree_Stats kdtree_calculate_stats(const KdTree& kdtree)
{
    KdTree_Stats stats;
    stats.nodes_size = kdtree.nodes.size() * sizeof(KdNode);
    stats.indices_size = kdtree.primitive_indices.size() * sizeof(uint32_t);
    stats.node_count = (uint32_t)kdtree.nodes.size();

    // Compute max depth (code should match computations from kdtree_builder).
    uint32_t primitive_count = kdtree.get_primitive_count();
    stats.max_depth_limit = std::lround(8.0 + 1.3 * std::floor(std::log2(primitive_count)));
    stats.max_depth_limit = std::min(stats.max_depth_limit, KdTree::max_traversal_depth);

    // Collect leaf count, primitives per leaf.
    uint64_t primitive_per_leaf_accumulated = 0;
    for (KdNode node : kdtree.nodes) {
        if (node.is_leaf()) {
            const uint32_t pc = node.get_primitive_count();
            if (pc == 0) {
                stats.empty_node_count++;
            }
            else {
                primitive_per_leaf_accumulated += pc;
                stats.leaf_count++;
                stats.leaf_primitives_max = std::max(stats.leaf_primitives_max, pc);
                if (pc <= 16)
                    stats.leaves_with_normal_primitive_count[pc - 1]++;
                else if (pc <= 32)
                    stats.leaves_with_large_primitive_count++;
                else
                    stats.leaves_with_huge_primitive_count++;
            }
        }
    }
    stats.leaf_primitives_mean = float(double(primitive_per_leaf_accumulated) / stats.leaf_count);

    // Compute depth of each leaf node.
    std::vector<uint8_t> leaf_depth_values;
    uint64_t max_depth_primitive_count_accumulated = 0;
    {
        struct Depth_Info {
            uint32_t node_index = 0;
            uint8_t depth = 0;
        };
        std::vector<Depth_Info> depth_info{ Depth_Info{0, 0} };
        for (size_t i = 0; i < depth_info.size(); i++) {
            uint32_t node_index = depth_info[i].node_index;
            uint8_t depth = depth_info[i].depth;
            KdNode node = kdtree.nodes[node_index];

            if (depth == stats.max_depth_limit) {
                ASSERT(node.is_leaf());
                if (node.get_primitive_count() > 0) {
                    stats.max_depth_leaf_count++;
                    max_depth_primitive_count_accumulated += node.get_primitive_count();
                }
            }

            if (node.is_leaf()) {
                if (node.get_primitive_count() > 0)
                    leaf_depth_values.push_back(depth);
            }
            else {
                uint32_t below_child_index = node_index + 1;
                depth_info.push_back({ below_child_index, uint8_t(depth + 1) });

                uint32_t above_child_index = node.get_above_child();
                depth_info.push_back({ above_child_index, uint8_t(depth + 1) });
            }
        }
    }
    if (stats.max_depth_leaf_count > 0)
        stats.max_depth_leaf_primitives_mean = float(double(max_depth_primitive_count_accumulated) / stats.max_depth_leaf_count);

    // Leaf depth mean/stddev.
    uint64_t leaf_depth_accumulated = std::accumulate(leaf_depth_values.cbegin(), leaf_depth_values.cend(), uint64_t(0));
    stats.leaf_depth_mean = float(double(leaf_depth_accumulated) / stats.leaf_count);
    double accum = 0.0;
    for (uint8_t depth : leaf_depth_values) {
        float diff = (float)depth - stats.leaf_depth_mean;
        accum += diff * diff;
    }
    stats.leaf_depth_std_dev = float(std::sqrt(accum / stats.leaf_count));
    return stats;
}

std::vector<uint32_t> kdtree_calculate_path_to_node(const KdTree& kdtree, uint32_t node_index)
{
    ASSERT(node_index >= 0 && node_index < kdtree.nodes.size());
    std::map<uint32_t, uint32_t> parent_map;
    for (uint32_t i = 0; i < uint32_t(kdtree.nodes.size()); i++) {
        if (!kdtree.nodes[i].is_leaf()) {
            uint32_t below_child = i + 1;
            uint32_t above_child = kdtree.nodes[i].get_above_child();
            parent_map[below_child] = i;
            parent_map[above_child] = i;
        }
    }
    std::vector<uint32_t> path{ node_index };
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
    auto get_percentage = [](uint64_t part, uint64_t total) { return float(double(part) / double(total)) * 100.f; };

    uint64_t size_in_bytes = nodes_size + indices_size;
    float size_in_mb = size_in_bytes / (1024.f * 1024.f);
    float nodes_size_percentage = get_percentage(nodes_size, size_in_bytes);
    float incides_size_percentage = std::max(0.f, 100.f - nodes_size_percentage);

    float leaf_nodes_percentage = get_percentage(leaf_count, node_count);
    float empty_nodes_percentage = get_percentage(empty_node_count, node_count);
    float interior_nodes_percentage = std::max(0.f, 100.f - leaf_nodes_percentage - empty_nodes_percentage);

    float max_depth_leaves_percentage = get_percentage(max_depth_leaf_count, leaf_count);
    float leaves_one_primitive_percentage = get_percentage(leaves_with_normal_primitive_count[0], leaf_count);
    float large_leaves_percentage = get_percentage(leaves_with_large_primitive_count, leaf_count);
    float huge_leaves_percentage = get_percentage(leaves_with_huge_primitive_count, leaf_count);

    uint32_t leaves_with_1_4_primitives = 0;
    for (int i = 0; i < 4; i++) leaves_with_1_4_primitives += leaves_with_normal_primitive_count[i];
    float leaves_1_4_percentage = get_percentage(leaves_with_1_4_primitives, leaf_count);

    uint32_t leaves_with_5_8_primitives = 0;
    for (int i = 4; i < 8; i++) leaves_with_5_8_primitives += leaves_with_normal_primitive_count[i];
    float leaves_5_8_percentage = get_percentage(leaves_with_5_8_primitives, leaf_count);

    uint32_t leaves_with_9_16_primitives = 0;
    for (int i = 8; i < 16; i++) leaves_with_9_16_primitives += leaves_with_normal_primitive_count[i];
    float leaves_9_16_percentage = get_percentage(leaves_with_9_16_primitives, leaf_count);

    printf("KdTree information\n");
    printf("------------------------\n");
    printf("kdtree size                     %.2f MB (%" PRIu64 " bytes)\n", size_in_mb, size_in_bytes);
    printf("nodes/indices memory ratio      nodes %.1f%%, indices %.1f%%\n", nodes_size_percentage, incides_size_percentage);
    printf("node count                      %u\n", node_count);
    printf("leaf count                      %u\n", leaf_count);
    printf("empty node count                %u\n", empty_node_count);
    printf("node type ratios                interior %.2f%%, leaves %.2f%%, empty %.2f%%\n",
        interior_nodes_percentage, leaf_nodes_percentage, empty_nodes_percentage);
    printf("leaf depth mean                 %.2f\n", leaf_depth_mean);
    printf("leaf depth std dev              %.2f\n", leaf_depth_std_dev);
    printf("leaf primitives mean            %.2f\n", leaf_primitives_mean);
    printf("leaf primitives max             %u\n", leaf_primitives_max);
    printf("max depth limit                 %u\n", max_depth_limit);
    printf("max depth leaf count            %u (%.2f%%)\n", max_depth_leaf_count, max_depth_leaves_percentage);
    printf("max depth leaf primitives mean  %.2f\n", max_depth_leaf_primitives_mean);
    printf("leaves with 1 primitive         %.2f%%\n", leaves_one_primitive_percentage);
    printf("leaves with 1-4 primitives      %.2f%%\n", leaves_1_4_percentage);
    printf("leaves with 5-8 primitives      %.2f%%\n", leaves_5_8_percentage);
    printf("leaves with 9-16 primitives     %.2f%%\n", leaves_9_16_percentage);
    printf("leaves with 17-32 primitives    %.2f%% (%u)\n", large_leaves_percentage, leaves_with_large_primitive_count);
    printf("leaves with > 32 primitives     %.2f%% (%u)\n", huge_leaves_percentage, leaves_with_huge_primitive_count);
    printf("\n");
}
