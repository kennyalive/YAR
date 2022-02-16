#pragma once

// 128 bytes (2 cache lines) tile.
// Single tile allows to store 4 level deep kdtree subtrees (max nodes per subtree = 1 + 2 + 4 + 8 = 15).
// _________________________________________________
// Level 0 |                  |0|                    |
// Level 1 |        |1|                  |2|         |
// Level 2 |   |3|      |4|        |5|        |6|    |
// Level 3 | |7| |8|  |9| |10|  |11| |12|  |13| |14| |
// __________________________________________________
//
struct KdTile {
    // 4 bytes: 2 bits per node for 15 nodes + 2 unused bits
    // If node's 2 bit field != 3 then it's an interior node, otherwise it's a leaf.
    // Additionally, for interior node, the value of 2 bit field (0, 1 or 2) defines the index of the split axis.
    uint32_t flags;

    // 60 bytes: split positions, used only by the interior nodes.
    float split_positions[15];

    // 64 bytes.
    // For 3rd level interior node, each pair of uint32 stores indices of the left and right "child tiles".
    // For leaf node (at most 8 leaves per tile), each pair of uint32 stores primitive count (first uint32) and
    // primitive index/offset (second uint32).
    uint32_t data[16];
};
static_assert(sizeof(KdTile) == 64 * 2);

struct KdTree;

std::vector<KdTile> convert_kdtree_nodes_to_tiled_layout(const KdTree& kdtree);
