#pragma once

#include "kdtree.h"

struct KdTree_Build_Params {
    float   intersection_cost = 80;
    float   traversal_cost = 1;
    float   empty_bonus = 0.3f;
    int     max_depth = -1;
    bool    split_along_the_longest_axis = false;
    int     leaf_triangles_limit = 2; // the actual amout of leaf triangles can be larger
    bool    split_clipping = true;
};

Mesh_KdTree build_kdtree(const Triangle_Mesh& mesh, const KdTree_Build_Params& build_params = KdTree_Build_Params());
TwoLevel_KdTree build_kdtree(const std::vector<Mesh_KdTree>& kdtrees, const KdTree_Build_Params& build_params = KdTree_Build_Params());
