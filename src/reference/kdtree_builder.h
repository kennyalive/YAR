#pragma once

#include "kdtree.h"

struct KdTree_Build_Params {
    int leaf_primitive_limit = 2; // the actual amount of leaf primitives can be larger
};

// Builds kdtree for a triangle mesh.
KdTree build_triangle_mesh_kdtree(const Triangle_Mesh_Geometry_Data* triangle_mesh_geometry_data, const KdTree_Build_Params& params);

// Builds kdtree that represents the entire scene.
// The leaf nodes contain references to kdtrees associated with scene geometry.
KdTree build_scene_kdtree(const Scene_Geometry_Data* scene_geometry_data, const KdTree_Build_Params& params);
