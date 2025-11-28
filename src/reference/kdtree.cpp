#include "std.h"
#include "lib/common.h"
#include "kdtree.h"

#include "image_texture.h"
#include "intersection.h"

#include "lib/scene_object.h"

constexpr int max_traversal_depth = 40;

static void intersect_triangle_mesh_geometry_data(const Ray& ray, const void* geometry_data, uint32_t primitive_index, Intersection& intersection)
{
    auto data = static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data);
    Vector3 p[3];
    data->mesh->get_positions(primitive_index, p);

    Vector3 b;
    float t = intersect_triangle_watertight(ray, p[0], p[1], p[2], &b);

    if (t < intersection.t) {
        // Check for degenerate triangle.
        Vector3 normal_direction = cross(p[1] - p[0], p[2] - p[0]);
        if (normal_direction.length_squared() == 0.f)
            return;
        
        // Do alpha test.
        if (data->alpha_texture != nullptr) {
            ASSERT(!data->mesh->uvs.empty());
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

static bool intersect_any_triangle_mesh_geometry_data(const Ray& ray, const void* geometry_data, uint32_t primitive_index, float ray_tmax)
{
    auto data = static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data);
    Vector3 p[3];
    data->mesh->get_positions(primitive_index, p);

    Vector3 b;
    float t = intersect_triangle_watertight(ray, p[0], p[1], p[2], &b);

    // Detect only intersections closer than ray_tmax
    if (t >= ray_tmax) {
        return false;
    }
    // Check for degenerate triangle.
    Vector3 normal_direction = cross(p[1] - p[0], p[2] - p[0]);
    if (normal_direction.length_squared() == 0.f) {
        return false;
    }
    // Do alpha test.
    if (data->alpha_texture != nullptr) {
        ASSERT(!data->mesh->uvs.empty());
        Vector2 uv = data->mesh->get_uv(primitive_index, b);
        ColorRGB alpha = data->alpha_texture->sample_bilinear(uv, 0, Wrap_Mode::repeat);
        if (alpha.r == 0.f) {
            return false; // skip this triangle
        }
    }
    return true;
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

static bool intersect_any_scene_geometry_data(const Ray& ray, const void* geometry_data, uint32_t primitive_index, float ray_tmax)
{
    auto data = static_cast<const Scene_Geometry_Data*>(geometry_data);

    ASSERT(primitive_index >= 0 && primitive_index < data->scene_objects->size());
    const Scene_Object* scene_object = &(*data->scene_objects)[primitive_index];
    Ray ray_in_object_space = transform_ray(scene_object->world_to_object_transform, ray);

    int offset = data->geometry_type_offsets[static_cast<int>(scene_object->geometry.type)];
    int kdtree_index = offset + scene_object->geometry.index;

    return (*data->kdtrees)[kdtree_index].intersect_any(ray_in_object_space, ray_tmax);
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
    any_intersector = &intersect_any_triangle_mesh_geometry_data;
    return true;
}

bool KdTree::set_geometry_data(const Scene_Geometry_Data* scene_geometry_data)
{
    uint64_t scene_kdtree_data_hash = compute_scene_kdtree_data_hash(*scene_geometry_data);
    if (geometry_data_hash != scene_kdtree_data_hash)
        return false;

    geometry_data = scene_geometry_data;
    intersector = &intersect_scene_geometry_data;
    any_intersector = &intersect_any_scene_geometry_data;
    return true;
}

uint32_t KdTree::get_primitive_count() const
{
    if (intersector == &intersect_triangle_mesh_geometry_data)
        return static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data)->mesh->get_triangle_count();
    else
        return (uint32_t)static_cast<const Scene_Geometry_Data*>(geometry_data)->scene_objects->size();
}

uint64_t KdTree::get_allocated_memory_size() const
{
    return (uint64_t)(nodes.size() * sizeof(KdNode));
}

int KdTree::get_max_depth_limit(uint32_t primitive_count)
{
    int depth = std::lround(8.0 + 1.3 * std::floor(std::log2(primitive_count)));
    return std::min(depth, max_traversal_depth);
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
    if (intersector == &intersect_triangle_mesh_geometry_data)
        primitive_count = static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data)->mesh->get_triangle_count();
    else
        primitive_count = (uint32_t)static_cast<const Scene_Geometry_Data*>(geometry_data)->scene_objects->size();

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

    const Vector3 inv_direction = Vector3(1.f) / ray.direction;

    struct Traversal_Info {
        const KdNode* node;
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
            prefetch(above_child);

            if (distance_to_split_plane != 0.0) { // general case
                const KdNode* first_child, * second_child;
                if (distance_to_split_plane > 0.0) {
                    first_child = below_child;
                    second_child = above_child;
                }
                else {
                    first_child = above_child;
                    second_child = below_child;
                }

                float t_split = distance_to_split_plane * inv_direction[axis];

                // distance_to_split_plane != 0 => t_split can't be zero.
                // This ensures that when ray direction is parallel to the splitting plane then
                // t_split can't be NaN (0 * inf results in NaN) but evaluates to +inf/-inf,
                // and using t_split == +inf/-inf in the following code produces correct result.
                ASSERT(t_split != 0.f);

                // Select node to traverse next.
                if (t_split > t_max || t_split < 0.0) {
                    node = first_child;
                }
                else if (t_split < t_min) { // 0 < t_split < t_min
                    node = second_child;
                }
                else { // t_min <= t_split <= t_max
                    ASSERT(traversal_stack_size < max_traversal_depth);
                    traversal_stack[traversal_stack_size].node = second_child;
                    traversal_stack[traversal_stack_size].t_max = t_max;
                    traversal_stack_size++;
                    node = first_child;
                    t_max = t_split;
                }
            }
            else { // special case, distance_to_split_plane == 0.0
                if (ray.direction[axis] > 0.0)
                    node = above_child;
                else
                    node = below_child;
            }
        }
        else { // leaf node
            const uint32_t* primitive_array = node->get_primitive_indices_array();
            const uint32_t primitive_count = node->get_primitive_count();

            for (uint32_t i = 0; i < primitive_count; i++) {
                intersector(ray, geometry_data, primitive_array[i], intersection);
            }

            if (traversal_stack_size == 0)
                break;

            --traversal_stack_size;

            node = traversal_stack[traversal_stack_size].node;
            t_min = t_max;
            t_max = traversal_stack[traversal_stack_size].t_max;
        }
    } // while (intersection.t > t_min)
    return intersection.t < ray_tmax;
#endif // !BRUTE_FORCE_INTERSECTION
}

bool KdTree::intersect_any(const Ray& ray, float ray_tmax) const
{
#ifdef BRUTE_FORCE_INTERSECTION
    uint32_t primitive_count = 0;
    if (any_intersector == &intersect_any_triangle_mesh_geometry_data)
        primitive_count = static_cast<const Triangle_Mesh_Geometry_Data*>(geometry_data)->mesh->get_triangle_count();
    else
        primitive_count = (uint32_t)static_cast<const Scene_Geometry_Data*>(geometry_data)->scene_objects->size();

    for (uint32_t i = 0; i < primitive_count; i++) {
        if (any_intersector(ray, geometry_data, i, ray_tmax)) {
            return true;
        }
    }
    return false;
#else
    float t_min, t_max; // parametric range for the ray's overlap with the current node

#if ENABLE_INVALID_FP_EXCEPTION
    if (!bounds.intersect_by_ray_without_NaNs(ray, &t_min, &t_max))
        return false;
#else
    if (!bounds.intersect_by_ray(ray, &t_min, &t_max))
        return false;
#endif

    const Vector3 inv_direction = Vector3(1.f) / ray.direction;

    struct Traversal_Info {
        const KdNode* node;
        float t_max;
    };
    Traversal_Info traversal_stack[max_traversal_depth];
    int traversal_stack_size = 0;

    const KdNode* node = &nodes[0];

    while (ray_tmax > t_min) {
        if (!node->is_leaf()) {
            const int axis = node->get_split_axis();
            const float distance_to_split_plane = node->get_split_position() - ray.origin[axis];

            const KdNode* below_child = node + 1;
            const KdNode* above_child = &nodes[node->get_above_child()];
            prefetch(above_child);

            if (distance_to_split_plane != 0.0) { // general case
                const KdNode* first_child, * second_child;
                if (distance_to_split_plane > 0.0) {
                    first_child = below_child;
                    second_child = above_child;
                }
                else {
                    first_child = above_child;
                    second_child = below_child;
                }

                float t_split = distance_to_split_plane * inv_direction[axis];

                // distance_to_split_plane != 0 => t_split can't be zero.
                // This ensures that when ray direction is parallel to the splitting plane then
                // t_split can't be NaN (0 * inf results in NaN) but evaluates to +inf/-inf,
                // and using t_split == +inf/-inf in the following code produces correct result.
                ASSERT(t_split != 0.f);

                // Select node to traverse next.
                if (t_split > t_max || t_split < 0.0) {
                    node = first_child;
                }
                else if (t_split < t_min) { // 0 < t_split < t_min
                    node = second_child;
                }
                else { // t_min <= t_split <= t_max
                    ASSERT(traversal_stack_size < max_traversal_depth);
                    traversal_stack[traversal_stack_size].node = second_child;
                    traversal_stack[traversal_stack_size].t_max = t_max;
                    traversal_stack_size++;
                    node = first_child;
                    t_max = t_split;
                }
            }
            else { // special case, distance_to_split_plane == 0.0
                if (ray.direction[axis] > 0.0)
                    node = above_child;
                else
                    node = below_child;
            }
        }
        else { // leaf node
            const uint32_t* primitive_array = node->get_primitive_indices_array();
            const uint32_t primitive_count = node->get_primitive_count();

            for (uint32_t i = 0; i < primitive_count; i++) {
                if (any_intersector(ray, geometry_data, primitive_array[i], ray_tmax)) {
                    return true;
                }
            }

            if (traversal_stack_size == 0)
                break;

            --traversal_stack_size;

            node = traversal_stack[traversal_stack_size].node;
            t_min = t_max;
            t_max = traversal_stack[traversal_stack_size].t_max;
        }
    } // while (ray_tmax > t_min)
    return false;
#endif // !BRUTE_FORCE_INTERSECTION
}
