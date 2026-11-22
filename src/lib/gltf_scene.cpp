#include "common.h"
#include "scene.h"

#define CGLTF_IMPLEMENTATION
#include "cgltf/cgltf.h"

template <typename Vector>
static void read_gltf_attribute(const cgltf_accessor& accessor, cgltf_type type, Array<Vector>& values)
{
    if (accessor.type != type) {
        error("Unexpected glTF vertex attribute type");
    }
    values.resize(accessor.count);
    for (size_t i = 0; i < accessor.count; i++) {
        float v[3];
        if (!cgltf_accessor_read_float(&accessor, i, v, 3)) {
            error("Failed to read glTF vertex attribute");
        }
        values[i] = Vector(v);
    }
}

static Geometry_Handle import_gltf_primitive(const cgltf_primitive& primitive, Scene& scene)
{
    if (primitive.type != cgltf_primitive_type_triangles || !primitive.indices) {
        error("Expected an indexed glTF triangle mesh in %s", scene.path.data());
    }
    const cgltf_accessor* positions = cgltf_find_accessor(&primitive, cgltf_attribute_type_position, 0);
    if (!positions || primitive.indices->count % 3 != 0) {
        error("Invalid glTF triangle mesh in %s", scene.path.data());
    }

    Triangle_Mesh mesh;
    read_gltf_attribute(*positions, cgltf_type_vec3, mesh.vertices);

    if (const cgltf_accessor* normals = cgltf_find_accessor(&primitive, cgltf_attribute_type_normal, 0)) {
        read_gltf_attribute(*normals, cgltf_type_vec3, mesh.normals);
    }
    if (const cgltf_accessor* uvs = cgltf_find_accessor(&primitive, cgltf_attribute_type_texcoord, 0)) {
        read_gltf_attribute(*uvs, cgltf_type_vec2, mesh.uvs);
    }

    mesh.indices.resize(primitive.indices->count);
    if (cgltf_accessor_unpack_indices(primitive.indices, mesh.indices.data(), 4, primitive.indices->count) !=
        primitive.indices->count) {
        error("Failed to read glTF triangle indices in %s", scene.path.data());
    }
    mesh.remove_degenerate_triangles();
    if (mesh.indices.empty()) {
        return Null_Geometry;
    }
    int mesh_index = (int)scene.geometries.triangle_meshes.size();
    scene.geometries.triangle_meshes.add(rvalue(mesh));
    return { Geometry_Type::triangle_mesh, mesh_index };
}

static Material_Handle import_gltf_material(const cgltf_material* material, Materials& materials)
{
    ColorRGB base_color = Color_White;
    float metallic = 1.f;
    float roughness = 1.f;
    float specular = 1.f;
    ColorRGB dielectric_r0(0.04f);
    if (material) {
        const cgltf_pbr_metallic_roughness& pbr = material->pbr_metallic_roughness;
        base_color = ColorRGB(pbr.base_color_factor);
        metallic = pbr.metallic_factor;
        roughness = pbr.roughness_factor;
        if (material->has_specular) {
            specular = material->specular.specular_factor;
            dielectric_r0 *= ColorRGB(material->specular.specular_color_factor) * specular;
        }
    }
    if (metallic == 0.f && specular == 0.f) {
        materials.diffuse.add().reflectance.set_constant(base_color);
        return { Material_Type::diffuse, (int)materials.diffuse.size() - 1 };
    }
    Coated_Diffuse_Material& coated = materials.coated_diffuse.add();
    coated.diffuse_reflectance.set_constant(base_color * (1.f - metallic));
    coated.r0.set_constant(lerp(dielectric_r0, base_color, metallic));
    coated.u_roughness.set_constant(max(roughness, 0.001f));
    coated.v_roughness = coated.u_roughness;
    return { Material_Type::coated_diffuse, (int)materials.coated_diffuse.size() - 1 };
}

static void import_gltf_area_light(ColorRGB radiance, Scene_Object& object, Scene& scene)
{
    // RAY samples area lights in world space
    if (!object.object_to_world_transform.is_identity()) {
        Triangle_Mesh mesh = scene.geometries.triangle_meshes[object.geometry.index];
        for (Vector3& vertex : mesh.vertices) {
            vertex = transform_point(object.object_to_world_transform, vertex);
        }
        const Matrix3x4& inverse = object.world_to_object_transform;
        for (Vector3& n : mesh.normals) {
            n = Vector3(dot(inverse.get_column(0), n), dot(inverse.get_column(1), n), dot(inverse.get_column(2), n)).normalized();
        }
        if (determinant_3x3(object.object_to_world_transform) < 0.f) {
            for (size_t i = 0; i < mesh.indices.size(); i += 3) {
                swap_values(mesh.indices[i + 1], mesh.indices[i + 2]);
            }
        }
        object.geometry.index = (int)scene.geometries.triangle_meshes.size();
        scene.geometries.triangle_meshes.add(rvalue(mesh));
        object.object_to_world_transform = Matrix3x4::identity;
        object.world_to_object_transform = Matrix3x4::identity;
    }
    Diffuse_Rectangular_Light rectangle;
    const Triangle_Mesh& mesh = scene.geometries.triangle_meshes[object.geometry.index];
    if (check_if_mesh_is_rectangle(mesh, rectangle.size, rectangle.light_to_world_transform)) {
        rectangle.emitted_radiance = radiance;
        object.area_light = { Light_Type::diffuse_rectangular, (int)scene.lights.diffuse_rectangular_lights.size() };
        scene.lights.diffuse_rectangular_lights.add(rectangle);
        return;
    }
    object.area_light = { Light_Type::diffuse_triangle_mesh, (int)scene.lights.diffuse_triangle_mesh_lights.size() };
    Diffuse_Triangle_Mesh_Light& light = scene.lights.diffuse_triangle_mesh_lights.add();
    light.triangle_mesh_index = (uint32_t)object.geometry.index;
    light.light_to_world_transform = object.object_to_world_transform;
    light.emitted_radiance = radiance;
}

static Matrix3x4 gltf_world_transform(const cgltf_node& node)
{
    float world[16];
    cgltf_node_transform_world(&node, world);
    Matrix3x4 transform;
    for (int column = 0; column < 4; column++) {
        transform.set_column(column, Vector3(world + column * 4));
    }
    return transform;
}

static void import_gltf_nodes(const cgltf_data& data, Scene& scene)
{
    if (data.scenes_count == 0) {
        error("No scene in glTF file %s", scene.path.data());
    }
    const cgltf_scene& gltf_scene = data.scene ? *data.scene : data.scenes[0];
    Array<cgltf_node*> nodes;
    nodes.add_all({ gltf_scene.nodes, gltf_scene.nodes_count });

    Array<Material_Handle> materials(data.materials_count + 1);
    Array<Array<Geometry_Handle>> mesh_geometries(data.meshes_count);
    for (size_t i = 0; i < nodes.size(); i++) {
        const cgltf_node& node = *nodes[i];
        nodes.add_all({ node.children, node.children_count });
        if (node.camera && node.camera->type == cgltf_camera_type_perspective && scene.view_points.empty()) {
            Matrix3x4 pose = gltf_world_transform(node);
            Vector3 back = pose.get_column(2);
            Vector3 right = cross(pose.get_column(1), back);
            if (right.length_squared() != 0.f) {
                back.normalize();
                right.normalize();
                pose.set_column(0, right);
                pose.set_column(1, cross(back, right));
                pose.set_column(2, back);
                scene.view_points.add(pose);
                scene.camera_fov_y = degrees(node.camera->data.perspective.yfov);
            }
        }
        if (!node.mesh) {
            continue;
        }
        Array<Geometry_Handle>& geometries = mesh_geometries[cgltf_mesh_index(&data, node.mesh)];
        if (geometries.empty()) {
            for (size_t j = 0; j < node.mesh->primitives_count; j++) {
                geometries.add(import_gltf_primitive(node.mesh->primitives[j], scene));
            }
        }
        const Matrix3x4 object_to_world = gltf_world_transform(node);
        if (determinant_3x3(object_to_world) == 0.f) {
            error("Non-invertible glTF mesh transform in %s", scene.path.data());
        }
        const Matrix3x4 world_to_object = get_inverse_transform(object_to_world);
        for (size_t j = 0; j < geometries.size(); j++) {
            Geometry_Handle geometry = geometries[j];
            if (geometry == Null_Geometry) {
                continue;
            }
            Scene_Object& object = scene.objects.add();
            object.geometry = geometry;
            object.object_to_world_transform = object_to_world;
            object.world_to_object_transform = world_to_object;

            const cgltf_material* material = node.mesh->primitives[j].material;
            if (material) {
                ColorRGB radiance(material->emissive_factor);
                if (material->has_emissive_strength) {
                    radiance *= material->emissive_strength.emissive_strength;
                }
                if (!radiance.is_black()) {
                    import_gltf_area_light(radiance, object, scene);
                    continue;
                }
            }
            size_t material_index = material ? cgltf_material_index(&data, material) : data.materials_count;
            if (materials[material_index] == Null_Material) {
                materials[material_index] = import_gltf_material(material, scene.materials);
            }
            object.material = materials[material_index];
        }
    }
}

void load_gltf_scene(Scene& scene)
{
    cgltf_options options{};
    cgltf_data* data = nullptr;
    cgltf_result result = cgltf_parse_file(&options, scene.path.data(), &data);

    if (result != cgltf_result_success) {
        error("Failed to parse glTF file %s (cgltf error %d)", scene.path.data(), result);
    }
    result = cgltf_load_buffers(&options, data, scene.path.data());
    if (result != cgltf_result_success) {
        cgltf_free(data);
        error("Failed to load glTF buffers for %s (cgltf error %d)", scene.path.data(), result);
    }
    result = cgltf_validate(data);
    if (result != cgltf_result_success) {
        cgltf_free(data);
        error("Invalid glTF file %s (cgltf error %d)", scene.path.data(), result);
    }

    import_gltf_nodes(*data, scene);
    printf("Loaded glTF (meshes: %zu, materials: %zu, cameras: %zu)\n", data->meshes_count, data->materials_count, data->cameras_count);
    cgltf_free(data);
}
