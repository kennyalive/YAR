#include "std.h"
#include "lib/common.h"
#include "kdtile.h"
#include "kdtree.h"

static int create_tile(uint32_t subtree_root_index, const std::vector<KdNode>& nodes, std::vector<KdTile>* tiles) {
    // Collect up to 15 nodes that will form a 'tile'.
    // Start with provided root KdNode and then access up to 3
    // levels below it by traversing in breadth-first order.
    const KdNode* subtree_nodes[15] = {};
    subtree_nodes[0] = &nodes[subtree_root_index];
    for (int i = 1; i < 15; i++) {
        const KdNode* parent = subtree_nodes[(i - 1) / 2];
        if (!parent || parent->is_leaf())
            continue;
 
        const bool is_left_child = (i & 1) != 0;
        uint32_t node_index;
        if (is_left_child) {
            uint32_t parent_index = uint32_t(parent - nodes.data());
            node_index = parent_index + 1;
        }
        else {
            node_index = parent->get_above_child();
        }
        subtree_nodes[i] = &nodes[node_index];
    }

    // Initialize tile based on collected subtree nodes.
    KdTile tile{};
    memset(tile.data, 255, 64);
    for (int i = 0; i < 15; i++) {
        if (!subtree_nodes[i])
            continue;

        tile.flags |= (subtree_nodes[i]->word0 & 3) << (2 * i);

        if (subtree_nodes[i]->is_leaf()) {
            // Determine slot in KdTile::data to which this leaf is projected to.
            // Check tile diagram in the header file to understand indexing logic.
            int level = log2_int(i + 1);
            int stride = 16 >> level;
            int level_first_index = (1 << level) - 1;
            int index_in_level = i - level_first_index;
            int offset = index_in_level * stride;

            ASSERT(tile.data[offset + 0] == UINT32_MAX);
            tile.data[offset + 0] = subtree_nodes[i]->get_primitive_count();
            ASSERT(tile.data[offset + 1] == UINT32_MAX);
            tile.data[offset + 1] = subtree_nodes[i]->get_index();
        }
        else {
            tile.split_positions[i] = subtree_nodes[i]->get_split_position();
        }
    }
    int this_tile_index = (int)tiles->size();
    tiles->push_back(tile);

    // Iterate over 3rd level interior nodes and create tiles for their children.
    for (int i = 7; i < 15; i++) {
        if (!subtree_nodes[i] || subtree_nodes[i]->is_leaf())
            continue;

        uint32_t left_child_index = uint32_t(subtree_nodes[i] - nodes.data()) + 1;
        int left_child_tile_index = create_tile(left_child_index, nodes, tiles);
        int right_child_tile_index = create_tile(subtree_nodes[i]->get_above_child(), nodes, tiles);

        int offset = (i - 7) * 2;
        ASSERT((*tiles)[this_tile_index].data[offset + 0] == UINT32_MAX);
        (*tiles)[this_tile_index].data[offset + 0] = left_child_tile_index;
        ASSERT((*tiles)[this_tile_index].data[offset + 1] == UINT32_MAX);
        (*tiles)[this_tile_index].data[offset + 1] = right_child_tile_index;
    }
    return this_tile_index;
}

std::vector<KdTile> convert_kdtree_nodes_to_tiled_layout(const KdTree& kdtree) {
    std::vector<KdTile> tiles;
    create_tile(0, kdtree.nodes, &tiles);
    return tiles;
}
