#include "std.h"
#include "lib/common.h"
#include "kdtile.h"
#include "kdtree.h"

constexpr int max_cache_lines_per_tile = 2;

// Final tile size is not fixed and can use smaller number of cache lines than max_cache_lines_per_tile.
// (but always multiple of cache line size). Varying tile size helps to deal with situation when
// we don't have enough nodes (because of leaves) to fill all max_cache_lines_per_tile.
constexpr int max_tile_size = max_cache_lines_per_tile * cache_line_size;
static_assert(max_tile_size <= 256); // we use 1 byte offsets for locations inside a tile

static int get_child_information_field_size(const KdNode* child_node, bool is_child_external)
{
    // Only interior nodes can be marked as external (i.e. located in external, non-current tile).
    ASSERT(!is_child_external || child_node->is_interior());

    if (child_node->is_empty())
        return 0; // no additional data for empty nodes

    if (child_node->is_leaf())
        return 5; // 1 byte for primitive count, 4 bytes for primitive index or offset

    if (is_child_external) {
        // 4 byte index of the cache line that starts external tile.
        // The referenced child node is the first node in that tile.
        return 4;
    }
    else {
        // Offset within the current tile that defines location of the child node.
        // Assumes that tile size <= 256 bytes
        return 1;
    }
}

static int get_tile_node_size(const KdNode* left_child, bool is_left_child_external, const KdNode* right_child, bool is_right_child_external)
{
    return
        1 /* metadata byte*/ +
        4 /* float32 split position */ +
        get_child_information_field_size(left_child, is_left_child_external) +
        get_child_information_field_size(right_child, is_right_child_external);
}

namespace {
struct Node_Info {
    bool active = false;
    int parent_info_index = -1;

    const KdNode* node = nullptr;
    const KdNode* parent = nullptr;
    const KdNode* left_child = nullptr;
    const KdNode* right_child = nullptr;

    // whether a child node should be placed in another tile
    bool is_left_child_external = false;
    bool is_right_child_external = false;

    // offsets of child nodes within the current tile (only for non-external interior nodes)
    int left_child_offset = -1; 
    int right_child_offset = - 1;
}; }

static std::vector<Node_Info> create_tile_layout(const KdNode* subtree_root, const std::vector<KdNode>& nodes)
{
    std::vector<Node_Info> layout;

    auto add_node = [&nodes, &layout](const KdNode* node, const KdNode* parent, int parent_info_index) {
        ASSERT(!node->is_leaf());
        layout.push_back(Node_Info{});
        Node_Info& node_info  = layout.back();

        node_info.active = false;
        node_info.parent_info_index = parent_info_index;

        node_info.node = node;
        node_info.parent = parent;
        node_info.left_child = node + 1;
        node_info.right_child = &nodes[node->get_above_child()];

        node_info.is_left_child_external = false;
        node_info.is_right_child_external = false;

        node_info.left_child_offset = -1;
        node_info.right_child_offset = -1;

        // It should be at least one not-empty child, otherwise current node should not exist
        ASSERT(!node_info.left_child->is_empty() || !node_info.right_child->is_empty());
    };
    add_node(subtree_root, nullptr, -1);

    int current_size = 0; // current tile size in bytes
    int index = 0;
    while (index < layout.size()) {
        Node_Info& current_node = layout[index];

        // Remove allocated child entries for current node, if any.
        // After we update node configuration we'll add child entries again based on updated configuration.
        if (current_node.active) {
            if (current_node.right_child->is_interior() && !current_node.is_right_child_external) {
                ASSERT(layout.back().parent == current_node.node);
                layout.pop_back();
            }
            if (current_node.left_child->is_interior() && !current_node.is_left_child_external) {
                ASSERT(layout.back().parent == current_node.node);
                layout.pop_back();
            }
        }

        // Get next configuration that allows to put node into available tile space.
        bool found_configuration = false;
        bool is_left_child_external = current_node.is_left_child_external;
        bool is_right_child_external = current_node.is_right_child_external;
        bool start_with_initial_state = !current_node.active;
        while (true) {
            if (start_with_initial_state) {
                // the first configuration is the initial child type values
                start_with_initial_state = false;
            }
            else if (current_node.right_child->is_interior() && !is_right_child_external) {
                is_right_child_external = true;
            }
            else if (current_node.left_child->is_interior() && !is_left_child_external) {
                is_left_child_external = true;
                if (current_node.right_child->is_interior()) {
                    is_right_child_external = false;
                }
            }
            else {
                // we failed to find configuration that allows to put node into a tile - there is
                // not enough space left for any configuration of the node.
                break;
            }
            int node_size = get_tile_node_size(
                current_node.left_child, is_left_child_external,
                current_node.right_child, is_right_child_external
            );

            if (current_size + node_size <= max_tile_size) {
                found_configuration = true;
                break;
            }
        }

        // If we have new node configuration then add child entries for it
        if (found_configuration) {
            current_node.active = true;
            current_node.is_left_child_external = is_left_child_external;
            current_node.is_right_child_external = is_right_child_external;

            if (current_node.parent_info_index != -1) {
                Node_Info& parent_info = layout[current_node.parent_info_index];
                ASSERT(current_node.node == parent_info.left_child || current_node.node == parent_info.right_child);
                ASSERT(current_size < 256);
                if (current_node.node == parent_info.left_child) {
                    parent_info.left_child_offset = current_size;
                }
                else {
                    ASSERT(current_node.node == parent_info.right_child);
                    parent_info.right_child_offset = current_size;
                }
            }

            current_size += get_tile_node_size(
                current_node.left_child, is_left_child_external,
                current_node.right_child, is_right_child_external
            );
            ASSERT(current_size <= max_tile_size);

            // NOTE: we use 'layout[index]' instead of 'current_node' shortcut because add_node invalidates references
            if (layout[index].left_child->is_interior() && !is_left_child_external) {
                add_node(layout[index].left_child, layout[index].node, index);
            }
            if (layout[index].right_child->is_interior() && !is_right_child_external) {
                add_node(layout[index].right_child, layout[index].node, index);
            }

            index++;
        }
        // otherwise, reset configuration state and go back to previous node
        else {
            current_node.active = false;
            current_node.is_left_child_external = false;
            current_node.is_right_child_external = false;
            current_node.left_child_offset = -1;
            current_node.right_child_offset = -1;

            ASSERT(index > 0);
            index--;

            current_size -= get_tile_node_size(
                layout[index].left_child, layout[index].is_left_child_external,
                layout[index].right_child, layout[index].is_right_child_external
            );
        }
    }
    return layout;
}

struct Tile_Request {
    // This node defines the root of the subtree that should be put into a single tile.
    // We put as much subtree nodes as possible into the tile until we reach tile size
    // limit or we run out of subtree nodes.
    const KdNode* subtree_root;

    // Offset in the 'tiles' byte array where to store 4 byte address of the newly created tile.
    // The tile address is a cache line index where the tile starts.
    uint64_t offset_of_tile_address_slot;
};

static KdTile_Child_Type get_child_type(const KdNode* child_node, bool is_child_external)
{
    // Only interior nodes can be marked as external (i.e. located in external, non-current tile).
    ASSERT(!is_child_external || child_node->is_interior());

    if (child_node->is_empty())
        return kdtile_child_type_empty;
    else if (child_node->is_leaf())
        return kdtile_child_type_leaf;
    else if (!is_child_external)
        return kdtile_child_type_node;
    else
        return kdtile_child_type_external_node;
}

static uint8_t get_tile_node_metadata(const Node_Info& node_info)
{
    KdTile_Child_Type left_child_type = get_child_type(node_info.left_child, node_info.is_left_child_external);
    KdTile_Child_Type right_child_type = get_child_type(node_info.right_child, node_info.is_right_child_external);
    uint8_t metadata = node_info.node->get_split_axis() | (left_child_type << 2) | (right_child_type << 4);
    return metadata;
}

static void append_tile_node(const Node_Info& node_info, std::vector<uint8_t>* tiles, 
    std::function<void (const Tile_Request&)> request_new_tile)
{
    ASSERT(!node_info.node->is_leaf());
    ASSERT(node_info.active);

    uint8_t metadata = get_tile_node_metadata(node_info);
    tiles->push_back(metadata);

    float split_position = node_info.node->get_split_position();
    uint8_t position_bytes[4];
    memcpy(position_bytes, &split_position, 4);
    tiles->insert(tiles->end(), position_bytes, position_bytes + 4);

    auto append_child_information = [tiles, &request_new_tile]
        (const KdNode* child_node, bool is_child_node_external, int child_offset)
    {
        if (child_node->is_empty()) {
            ; // empty nodes have no associated data
        }
        else if (child_node->is_leaf()) {
            uint32_t primitive_count = child_node->get_primitive_count();
            ASSERT(primitive_count < 256);
            tiles->push_back(uint8_t(primitive_count));

            uint32_t primitive_index_or_offset = child_node->get_index();
            uint8_t index_bytes[4];
            memcpy(index_bytes, &primitive_index_or_offset, 4);
            tiles->insert(tiles->end(), index_bytes, index_bytes + 4);
        }
        else if (!is_child_node_external) {
            ASSERT(child_offset != -1 && child_offset < 256);
            tiles->push_back(uint8_t(child_offset));
        }
        else {
            Tile_Request request;
            request.subtree_root = child_node;
            request.offset_of_tile_address_slot = (uint64_t)tiles->size();
            tiles->insert(tiles->end(), 4, uint8_t(0)); // reserve bytes for tile address
            request_new_tile(request);
        }
    };
    append_child_information(node_info.left_child, node_info.is_left_child_external, node_info.left_child_offset);
    append_child_information(node_info.right_child, node_info.is_right_child_external, node_info.right_child_offset);
}

static uint32_t create_tile(const KdNode* subtree_root, const std::vector<KdNode>& nodes, std::vector<uint8_t>* tiles)
{
    ASSERT(tiles->size() % cache_line_size == 0);
    const uint32_t tile_cache_line_index = uint32_t(tiles->size() / cache_line_size);

    static constexpr int max_tile_requests = 32;
    Tile_Request tile_requests[max_tile_requests]; // the tiles referenced from the current tile
    int tile_request_count = 0;

    auto add_tile_request = [&tile_request_count, &tile_requests](const Tile_Request& request) {
        ASSERT(tile_request_count < max_tile_requests);
        tile_requests[tile_request_count++] = request;
    };

    std::vector<Node_Info> layout = create_tile_layout(subtree_root, nodes);
    for (const Node_Info& node_info : layout) {
        append_tile_node(node_info, tiles, add_tile_request);
    }

    int padding_byte_count = (cache_line_size - tiles->size() % cache_line_size) % cache_line_size;
    if (padding_byte_count > 0) {
        tiles->insert(tiles->end(), padding_byte_count, 0);
    }

    // Create tiles references by the current tile and initialize references to 
    // created tiles by writing to corresponding address slots inside current tile.
    for (int i = 0; i < tile_request_count; i++) {
        uint32_t tile_cache_line_index = create_tile(tile_requests[i].subtree_root, nodes, tiles);
        uint8_t* dst = tiles->data() + tile_requests[i].offset_of_tile_address_slot;
        memcpy(dst, &tile_cache_line_index, 4);
    }

    return tile_cache_line_index;
}

std::vector<uint8_t> convert_kdtree_nodes_to_tiled_layout(const KdTree& kdtree)
{
    std::vector<uint8_t> tiles;
    create_tile(&kdtree.nodes[0], kdtree.nodes, &tiles);
    return tiles;
}
