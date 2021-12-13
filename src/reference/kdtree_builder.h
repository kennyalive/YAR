#pragma once

#include "kdtree.h"

struct Triangle_Mesh;

struct KdTree_Build_Params {
    float intersection_cost = 80;
    float traversal_cost = 1;
    float empty_bonus = 0.3f;
    int max_depth = -1;
    bool split_along_the_longest_axis = false;
    int leaf_primitive_limit = 2; // the actual amout of leaf primitives can be larger
    bool split_clipping = true;
};

// Builds kdtree for a triangle mesh.
KdTree build_triangle_mesh_kdtree(const Triangle_Mesh* mesh, const KdTree_Build_Params& params);

// Builds kdtree that represents the entire scene.
// The leaf nodes contain references to kdtrees associated with scene geometry.
KdTree build_scene_kdtree(const Scene_KdTree_Data* scene_kdtree_data, const KdTree_Build_Params& params);
