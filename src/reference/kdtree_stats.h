#pragma once

struct KdTree_Stats {
    uint64_t nodes_size = 0;
    uint64_t indices_size = 0;
    uint32_t node_count = 0;
    uint32_t empty_node_count = 0;
    uint32_t leaf_count = 0;

    uint32_t leaves_with_normal_primitive_count[16] = {}; // 1-16 primitives per node
    uint32_t leaves_with_large_primitive_count = 0; //17-32
    uint32_t leaves_with_huge_primitive_count = 0; // > 32

    float leaf_depth_mean = 0.f;
    float leaf_depth_std_dev = 0.f;
    float leaf_primitives_mean = 0.f;

    int max_depth_limit = 0;
    uint32_t max_depth_leaf_count = 0;
    float max_depth_leaf_primitives_mean = 0;

    void print();
};

struct KdTree;

KdTree_Stats kdtree_calculate_stats(const KdTree& kdtree);
std::vector<uint32_t> kdtree_calculate_path_to_node(const KdTree& kdtree, uint32_t node_index);
