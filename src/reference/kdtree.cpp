#include "std.h"
#include "lib/common.h"
#include "kdtree.h"

#include "image_texture.h"
#include "intersection.h"

#include "lib/scene_object.h"

static void intersect_triangle_mesh_geometry_data(const Ray& ray, const void* geometry_data, uint32_t primitive_index, Intersection& intersection)
{
    auto data = static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data);
    Vector3 p0, p1, p2;
    data->mesh->get_triangle(primitive_index, p0, p1, p2);

    Vector3 b;
    float t = intersect_triangle_watertight(ray, p0, p1, p2, &b);

    const bool found_closer_intersection = t < intersection.t;

    // When few primitives overlap at intersection point then select a primitive
    // with a smaller index to have deterministic behavior with regard to using
    // different kdtree version which can list primitives in a different order.
    const bool primitive_overlap_resolve = (t == intersection.t && primitive_index < intersection.triangle_intersection.triangle_index);

    if (found_closer_intersection || primitive_overlap_resolve) {
        // Do alpha test.
        if (data->alpha_texture != nullptr) {
            Vector2 uv = data->mesh->get_uv(primitive_index, b);
            ColorRGB alpha = data->alpha_texture->sample_bilinear(uv, 0, Wrap_Mode::repeat);
            if (alpha.r == 0.f)
                return; // skip this triangle
        }
        intersection.t = t;
        intersection.geometry_type = Geometry_Type::triangle_mesh;
        intersection.triangle_intersection.barycentrics = b;
        intersection.triangle_intersection.mesh = data->mesh;
        intersection.triangle_intersection.triangle_index = primitive_index;
    }
}

static void intersect_scene_geometry_data(const Ray& ray, const void* geometry_data, uint32_t primitive_index, Intersection& intersection)
{
    auto data = static_cast<const Scene_Geometry_Data*>(geometry_data);

    ASSERT(primitive_index >= 0 && primitive_index < data->scene_objects->size());
    const Scene_Object* scene_object = &(*data->scene_objects)[primitive_index];
    Ray ray_in_object_space = transform_ray(scene_object->world_to_object_transform, ray);

    int offset = data->geometry_type_offsets[static_cast<int>(scene_object->geometry.type)];
    int kdtree_index = offset + scene_object->geometry.index;

    if ((*data->kdtrees)[kdtree_index].intersect(ray_in_object_space, intersection)) {
        intersection.scene_object = scene_object;
    }
}

KdTree KdTree::load(const std::string& file_name)
{
    std::ifstream file(file_name, std::ios_base::in | std::ios_base::binary);
    if (!file)
        error("KdTree::load: failed to open file: %s", file_name.c_str());

    KdTree kdtree;

    // bounds
    static_assert(sizeof(Bounding_Box) == 2 * 3*sizeof(float));
    file.read(reinterpret_cast<char*>(&kdtree.bounds), sizeof(Bounding_Box));

    // geometry hash
    static_assert(sizeof(KdTree::geometry_data_hash) == sizeof(uint64_t));
    file.read(reinterpret_cast<char*>(&kdtree.geometry_data_hash), sizeof(uint64_t));

    // nodes
    uint32_t node_count = 0;
    file.read(reinterpret_cast<char*>(&node_count), 4);
    kdtree.nodes.resize(node_count);
    size_t nodes_byte_count = node_count * sizeof(KdNode);
    file.read(reinterpret_cast<char*>(kdtree.nodes.data()), nodes_byte_count);

    // primitive indices
    uint32_t index_count = 0;
    file.read(reinterpret_cast<char*>(&index_count), 4);
    kdtree.primitive_indices.resize(index_count);
    size_t indices_byte_count = index_count * 4;
    file.read(reinterpret_cast<char*>(kdtree.primitive_indices.data()), indices_byte_count);

    if (file.fail())
        error("KdTree::load: failed to read kdtree data: %s", file_name.c_str());

    return kdtree;
}

void KdTree::save(const std::string& file_name) const
{
    std::ofstream file(file_name, std::ios_base::out | std::ios_base::binary);
    if (!file)
        error("KdTree::save: failed to open file for writing: %s", file_name.c_str());

    // bounds
    file.write(reinterpret_cast<const char*>(&bounds), sizeof(Bounding_Box));

    // geometry hash
    file.write(reinterpret_cast<const char*>(&geometry_data_hash), sizeof(uint64_t));

    // nodes
    uint32_t node_count = (uint32_t)nodes.size();
    file.write(reinterpret_cast<const char*>(&node_count), sizeof(uint32_t));
    size_t nodes_byte_count = node_count * sizeof(KdNode);
    file.write(reinterpret_cast<const char*>(nodes.data()), nodes_byte_count);

    // primitive indices
    uint32_t index_count = (uint32_t)primitive_indices.size();
    file.write(reinterpret_cast<const char*>(&index_count), sizeof(uint32_t));
    size_t indices_byte_count = index_count * 4;
    file.write(reinterpret_cast<const char*>(primitive_indices.data()), indices_byte_count);

    if (file.fail())
        error("KdTree::save: failed to write kdtree data: %s", file_name.c_str());
}

bool KdTree::set_geometry_data(const Triangle_Mesh_Geometry_Data* triangle_mesh_geometry_data)
{
    uint64_t mesh_hash = compute_triangle_mesh_hash(*triangle_mesh_geometry_data->mesh);
    if (geometry_data_hash != mesh_hash)
        return false;

    geometry_data = triangle_mesh_geometry_data;
    intersector = &intersect_triangle_mesh_geometry_data;
    return true;
}

bool KdTree::set_geometry_data(const Scene_Geometry_Data* scene_geometry_data)
{
    uint64_t scene_kdtree_data_hash = compute_scene_kdtree_data_hash(*scene_geometry_data);
    if (geometry_data_hash != scene_kdtree_data_hash)
        return false;

    geometry_data = scene_geometry_data;
    intersector = &intersect_scene_geometry_data;
    return true;
}

uint32_t KdTree::get_primitive_count() const
{
    if (intersector == &intersect_triangle_mesh_geometry_data)
        return static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data)->mesh->get_triangle_count();
    else
        return (uint32_t)static_cast<const Scene_Geometry_Data*>(geometry_data)->scene_objects->size();
}

uint64_t KdTree::compute_triangle_mesh_hash(const Triangle_Mesh& mesh)
{
    // TODO: implement me
    return 0;
}

uint64_t KdTree::compute_scene_kdtree_data_hash(const Scene_Geometry_Data& scene_geometry_data)
{
    // TODO: implement me
    return 0;
}

//#define BRUTE_FORCE_INTERSECTION

bool KdTree::intersect(const Ray& ray, Intersection& intersection) const
{
#ifdef BRUTE_FORCE_INTERSECTION
    uint32_t primitive_count = 0;
    if (intersector == &intersect_triangle_mesh_kdtree_leaf_primitive)
        primitive_count = static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data)->mesh->get_triangle_count();
    else
        primitive_count = (uint32_t)static_cast<const Scene_KdTree_Data*>(geometry_data)->scene_objects->size();

    float tmax = intersection.t;
    for (uint32_t i = 0; i < primitive_count; i++) {
        intersector(ray, geometry_data, i, intersection);
    }
    return intersection.t < tmax;
#else
    float t_min, t_max; // parametric range for the ray's overlap with the current node

#if ENABLE_INVALID_FP_EXCEPTION
    if (!bounds.intersect_by_ray_without_NaNs(ray, &t_min, &t_max))
        return false;
#else
    if (!bounds.intersect_by_ray(ray, &t_min, &t_max))
        return false;
#endif

    struct Traversal_Info {
        const KdNode* node;
        float t_min;
        float t_max;
    };
    Traversal_Info traversal_stack[max_traversal_depth];
    int traversal_stack_size = 0;

    const KdNode* node = &nodes[0];
    const float ray_tmax = intersection.t;

    while (intersection.t > t_min) {
        if (!node->is_leaf()) {
            const int axis = node->get_split_axis();
            const float distance_to_split_plane = node->get_split_position() - ray.origin[axis];

            const KdNode* below_child = node + 1;
            const KdNode* above_child = &nodes[node->get_above_child()];

            // TODO: test this more on complex scenes. This improves performance on landscape scene (3.5%)
            // but for simpler scenes it's not always a win.
            //_mm_prefetch((const char*)above_child, _MM_HINT_T0);

            if (distance_to_split_plane != 0.0) { // general case
                const KdNode *first_child, *second_child;
                if (distance_to_split_plane > 0.0) {
                    first_child = below_child;
                    second_child = above_child;
                }
                else {
                    first_child = above_child;
                    second_child = below_child;
                }

                // Select node to traverse next.
                float t_split = distance_to_split_plane / ray.direction[axis]; // != 0 because distance_to_split_plane != 0
                if (t_split >= t_max || t_split < 0.0) {
                    node = first_child;
                }
                else if (t_split <= t_min) { // 0 < t_split <= t_min
                    node = second_child;
                }
                else { // t_min < t_split < t_max
                    ASSERT(traversal_stack_size < max_traversal_depth);
                    traversal_stack[traversal_stack_size++] = {second_child, t_split, t_max};
                    node = first_child;
                    t_max = t_split;
                }
            }
            else { // special case, distance_to_split_plane == 0.0
                if (ray.direction[axis] > 0.0) {
                    if (t_min > 0.0)
                        node = above_child;
                    else { // t_min == 0.0
                        ASSERT(traversal_stack_size < max_traversal_depth);
                        traversal_stack[traversal_stack_size++] = {above_child, 0.0, t_max};
                        // check single point [0.0, 0.0]
                        node = below_child;
                        t_max = 0.0;
                    }
                }
                else if (ray.direction[axis] < 0.0) {
                    if (t_min > 0.0)
                        node = below_child;
                    else { // t_min == 0.0
                        ASSERT(traversal_stack_size < max_traversal_depth);
                        traversal_stack[traversal_stack_size++] = {below_child, 0.0, t_max};
                        // check single point [0.0, 0.0]
                        node = above_child;
                        t_max = 0.0;
                    }
                }
                else { // ray.direction[axis] == 0.0
                    // for both nodes check [t_min, t_max] range
                    ASSERT(traversal_stack_size < max_traversal_depth);
                    traversal_stack[traversal_stack_size++] = {above_child, t_min, t_max};
                    node = below_child;
                }
            }
        }
        else { // leaf node
            if (node->get_primitive_count() == 1) {
                intersector(ray, geometry_data, node->get_index(), intersection);
            }
            else {
                for (uint32_t i = 0; i < node->get_primitive_count(); i++) {
                    uint32_t primitive_index = primitive_indices[node->get_index() + i];
                    intersector(ray, geometry_data, primitive_index, intersection);
                }
            }

            if (traversal_stack_size == 0)
                break;

            // Almost correct implementation is just: --traversal_stack_size.
            // We need to scan the entire stack to handle the case when distance_to_split_plane == 0.0 && ray.direction[axis] == 0.0.
            do {
                --traversal_stack_size;
            } while (traversal_stack_size > 0 && traversal_stack[traversal_stack_size].t_min >= intersection.t);

            node = traversal_stack[traversal_stack_size].node;
            t_min = traversal_stack[traversal_stack_size].t_min;
            t_max = traversal_stack[traversal_stack_size].t_max;
        }
    } // while (intersection.t > t_min)
    return intersection.t < ray_tmax;
#endif // !BRUTE_FORCE_INTERSECTION
}

bool KdTree::intersect_any(const Ray& ray, float tmax) const
{
    Intersection intersection{ tmax };
    return intersect(ray, intersection);
}
