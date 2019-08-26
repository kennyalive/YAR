#pragma once

#include "kdtree.h"

struct KdTree_Build_Params {
    float intersection_cost = 80;
    float traversal_cost = 1;
    float empty_bonus = 0.3f;
    int max_depth = -1;
    bool split_along_the_longest_axis = false;
    int leaf_primitive_limit = 2; // the actual amout of leaf primitives can be larger
    bool split_clipping = true;
};

Geometry_KdTree build_geometry_kdtree(const Geometries* geometries, Geometry_Handle geometry, const KdTree_Build_Params& build_params = KdTree_Build_Params());
Scene_KdTree build_scene_kdtree(const Scene* scene, std::vector<Geometry_KdTree>&& kdtrees, const KdTree_Build_Params& build_params = KdTree_Build_Params());

