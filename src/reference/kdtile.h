#pragma once

constexpr int cache_line_size = 64;

// All nodes in the tiled layout are 'interior' nodes. We don't allocate
// dedicated nodes to store leaf related information or, for empty nodes,
// we don't allocate dedicated nodes just to mark them as empty.
//
// If the left or the right child of the current node terminates traversal then
// all leaf related information is stored in the current node.
//
// One benefit of this approach is that we avoid allocation of real memory for
// 'empty nodes' (i.e. leaves that have no primitives). Also we avoid that last
// jump to the leaf node which might cause a cache miss but instead we grab all 
// leaf related information from the current node.
enum KdTile_Child_Type {
    kdtile_child_type_empty = 0, // ends traversal
    kdtile_child_type_leaf = 1, // ends traversal, leaf primitives information is stored in the current node
    kdtile_child_type_node = 2, // reference to a node from the current tile
    kdtile_child_type_external_node = 3 // reference to a node from a different tile
};

struct KdTree;
std::vector<uint8_t> convert_kdtree_nodes_to_tiled_layout(const KdTree& kdtree);
