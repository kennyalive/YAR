#pragma once

#include "kdtree.h"

struct Triangle_Mesh;

struct KdTree_Build_Params {
    float intersection_cost = 80;
    float traversal_cost = 1;
    float empty_bonus = 0.3f;
    int max_depth = -1;
    int leaf_primitive_limit = 2; // the actual amount of leaf primitives can be larger
};

// Builds kdtree for a triangle mesh.
KdTree build_triangle_mesh_kdtree(const Triangle_Mesh_Geometry_Data* triangle_mesh_geometry_data, const KdTree_Build_Params& params);

// Builds kdtree that represents the entire scene.
// The leaf nodes contain references to kdtrees associated with scene geometry.
KdTree build_scene_kdtree(const Scene_Geometry_Data* scene_geometry_data, const KdTree_Build_Params& params);
