#include "std.h"
#include "common.h"

#include "colorimetry.h"
#include "spectrum.h"
#include "scene.h"
#include "scene_loader.h"
#include "tessellation.h"
#include "yar_project.h"

#include "pbrtParser/Scene.h"
#include "pbrt-parser/impl/syntactic/Scene.h"

namespace {
struct Shape {
    Geometry_Handle geometry;
    Light_Handle area_light;
    Matrix3x4 transform = Matrix3x4::identity;
    // TODO: remove this field after we separate material from shape in pbrt parser
    Material_Handle material;
};
}

static Matrix3x4 to_matrix3x4(const pbrt::affine3f& pbrt_transform) {
    const pbrt::math::vec3f& pos = pbrt_transform.p;
    const pbrt::math::mat3f& rot = pbrt_transform.l;
    Matrix3x4 mat;
    mat.set_column(0, Vector3(&rot.vx.x));
    mat.set_column(1, Vector3(&rot.vy.x));
    mat.set_column(2, Vector3(&rot.vz.x));
    mat.set_column(3, Vector3(&pos.x));
    return mat;
}

static Sampled_Spectrum to_sampled_spectrum(const pbrt::Spectrum& pbrt_spectrum) {
    std::vector<float> lambdas(pbrt_spectrum.spd.size());
    std::vector<float> values(pbrt_spectrum.spd.size());
    for (int i = 0; i < int(pbrt_spectrum.spd.size()); i++) {
        lambdas[i] = pbrt_spectrum.spd[i].first;
        values[i] = pbrt_spectrum.spd[i].second;
    }
    return Sampled_Spectrum::from_tabulated_data(lambdas.data(), values.data(), (int)lambdas.size());
}

static bool check_if_mesh_is_rectangle(const Triangle_Mesh& mesh, Vector2& size, Matrix3x4& transform) {
    bool potentially_rectangle_topology =
        (mesh.vertices.size() == 4 || mesh.vertices.size() == 6) && mesh.indices.size() == 6;
    if (!potentially_rectangle_topology)
        return false;

    Vector3 p[3] = {
        mesh.vertices[mesh.indices[0]],
        mesh.vertices[mesh.indices[1]],
        mesh.vertices[mesh.indices[2]]
    };

    bool found_p3 = false;
    Vector3 p3;
    for (int i = 3; i < 6; i++) {
        Vector3 pp = mesh.vertices[mesh.indices[i]];

        // Looking for the vertex of the second triangle that is not shared with the first triangle.
        if ((pp - p[0]).length() < 1e-4f || (pp - p[1]).length() < 1e-4f || (pp - p[2]).length() < 1e-4f)
            continue;
        // We already found non-shared vertex and now we have one more. It's not a rectangle.
        if (found_p3)
            return false;

        found_p3 = true;
        p3 = pp;
    }
    if (!found_p3)
        return false;

    Vector3 v[3] = {
        p[1] - p[0],
        p[2] - p[1],
        p[0] - p[2]
    };
    Vector3 d[3] = {
        v[0].normalized(),
        v[1].normalized(),
        v[2].normalized()
    };

    int k = 0;
    for (; k < 3; k++) {
        if (std::abs(dot(d[k], d[(k+1)%3])) < 1e-4f) {
            break;
        }
    }
    if (k == 3) // we couldn't find a right angle in the first triangle, so entire shape is not a rectangle
        return false;

    Vector3 mid_point = (p[0] + p[1] + p[2] + p3) * 0.25f;
    Vector3 test_point = (p[k] + p[(k+2)%3]) * 0.5f;

    if ((mid_point - test_point).length() > 1e-4f)
        return false;

    Vector3 x_axis = d[k];
    Vector3 y_axis = d[(k+1)%3];
    Vector3 z_axis = cross(x_axis, y_axis);

    size.x = v[k].length();
    size.y = v[(k+1)%3].length();

    transform.set_column(0, x_axis);
    transform.set_column(1, y_axis);
    transform.set_column(2, z_axis);
    transform.set_column(3, mid_point);
    return true;
}

static RGB_Parameter import_pbrt_texture_rgb(const pbrt::Texture::SP pbrt_texture, Scene* scene) {
    RGB_Parameter param;

    if (auto image_texture = std::dynamic_pointer_cast<pbrt::ImageTexture>(pbrt_texture);
        image_texture != nullptr)
    {
        Texture_Descriptor texture_desc{
            .file_name = image_texture->fileName,
            .decode_srgb = image_texture->gamma
        };
        int texture_index = add_scene_texture(texture_desc, scene);
        set_texture_parameter(param, texture_index);

        param.u_scale = image_texture->uscale;
        param.v_scale = image_texture->vscale;
    }
    else if (auto constant_texture = std::dynamic_pointer_cast<pbrt::ConstantTexture>(pbrt_texture);
        constant_texture != nullptr)
    {
        set_constant_parameter(param, ColorRGB(&constant_texture->value.x));
    }
    else if (auto scale_texture = std::dynamic_pointer_cast<pbrt::ScaleTexture>(pbrt_texture);
        scale_texture != nullptr)
    {
        ASSERT(scale_texture->tex1 != nullptr);
        ASSERT(scale_texture->tex2 == nullptr);
        ASSERT(Vector3(&scale_texture->scale1.x) == Vector3(1));
        ASSERT(scale_texture->scale2.x == scale_texture->scale2.y && scale_texture->scale2.y == scale_texture->scale2.z);

        auto image_texture = std::dynamic_pointer_cast<pbrt::ImageTexture>(scale_texture->tex1);
        ASSERT(image_texture != nullptr);

        int texture_index = add_scene_texture(
            Texture_Descriptor{
                .file_name = image_texture->fileName,
                .scale = scale_texture->scale2.x,
            },
            scene
        );
        set_texture_parameter(param, texture_index);
        param.u_scale = image_texture->uscale;
        param.v_scale = image_texture->vscale;
    }
    else
        error("Unsupported pbrt texture type");

    return param;
}

static Float_Parameter import_pbrt_texture_float(const pbrt::Texture::SP pbrt_texture, Scene* scene) {
    Float_Parameter param;

    if (auto image_texture = std::dynamic_pointer_cast<pbrt::ImageTexture>(pbrt_texture);
        image_texture != nullptr)
    {
        Texture_Descriptor texture_desc{
            .file_name = image_texture->fileName,
            .decode_srgb = image_texture->gamma
        };
        int texture_index = add_scene_texture(texture_desc, scene);
        set_texture_parameter(param, texture_index);

        param.u_scale = image_texture->uscale;
        param.v_scale = image_texture->vscale;
    }
    else if (auto constant_texture = std::dynamic_pointer_cast<pbrt::ConstantTexture>(pbrt_texture);
        constant_texture != nullptr)
    {
        ColorRGB xyz = sRGB_to_XYZ(ColorRGB(&constant_texture->value.x));
        set_constant_parameter(param, xyz[1]);
    }
    else if (auto scale_texture = std::dynamic_pointer_cast<pbrt::ScaleTexture>(pbrt_texture);
        scale_texture != nullptr)
    {
        ASSERT(scale_texture->tex1 != nullptr);
        ASSERT(scale_texture->tex2 == nullptr);
        ASSERT(Vector3(&scale_texture->scale1.x) == Vector3(1));
        ASSERT(scale_texture->scale2.x == scale_texture->scale2.y && scale_texture->scale2.y == scale_texture->scale2.z);

        auto image_texture = std::dynamic_pointer_cast<pbrt::ImageTexture>(scale_texture->tex1);
        ASSERT(image_texture != nullptr);

        int texture_index = add_scene_texture(
            Texture_Descriptor{
                .file_name = image_texture->fileName,
                .scale = scale_texture->scale2.x,
            },
            scene
        );
        set_texture_parameter(param, texture_index);

        param.u_scale = image_texture->uscale;
        param.v_scale = image_texture->vscale;
    }
    else
        error("Unsupported pbrt texture type");

    return param;
}

template <Material_Type material_type, typename Material>
Material_Handle add_material(Materials& materials, const Material& material)
{
    std::vector<Material>* materials_of_given_type = nullptr;
    if constexpr (material_type == Material_Type::lambertian) {
        materials_of_given_type = &materials.lambertian;
    }
    else if constexpr (material_type == Material_Type::diffuse_transmission) {
        materials_of_given_type = &materials.diffuse_transmission;
    }
    else if constexpr (material_type == Material_Type::perfect_reflector) {
        materials_of_given_type = &materials.perfect_reflector;
    }
    else if constexpr (material_type == Material_Type::perfect_refractor) {
        materials_of_given_type = &materials.perfect_refractor;
    }
    else if constexpr (material_type == Material_Type::metal) {
        materials_of_given_type = &materials.metal;
    }
    else if constexpr (material_type == Material_Type::plastic) {
        materials_of_given_type = &materials.plastic;
    }
    else if constexpr (material_type == Material_Type::coated_diffuse) {
        materials_of_given_type = &materials.coated_diffuse;
    }
    else if constexpr (material_type == Material_Type::glass) {
        materials_of_given_type = &materials.glass;
    }
    else if constexpr (material_type == Material_Type::pbrt3_uber) {
        materials_of_given_type = &materials.pbrt3_uber;
    }
    else {
        static_assert(dependent_false_v<Material>, "add_material: Unexpected Material_Type");
    }

    // Check if we already have this material registered. If yes, then return the existing handle.
    for (size_t i = 0; i < materials_of_given_type->size(); i++) {
        if ((*materials_of_given_type)[i] == material)
            return Material_Handle{ material_type, (int)i };
    }
    // Add new material.
    materials_of_given_type->push_back(material);
    return Material_Handle{ material_type, (int)materials_of_given_type->size() - 1 };
}

static RGB_Parameter init_rgb_parameter_from_texture_or_constant(Scene* scene,
    pbrt::Texture::SP texture, const pbrt::vec3f& const_value)
{
    RGB_Parameter param;
    if (texture)
        param = import_pbrt_texture_rgb(texture, scene);
    else
        set_constant_parameter(param, ColorRGB(&const_value.x));
    return param;
}

static Float_Parameter init_float_parameter_from_texture_or_constant(Scene* scene,
    pbrt::Texture::SP texture, float const_value)
{
    Float_Parameter param;
    if (texture)
        param = import_pbrt_texture_float(texture, scene);
    else
        set_constant_parameter(param, const_value);
    return param;
}

static Material_Handle import_pbrt_material(const pbrt::Material::SP pbrt_material, Scene* scene)
{
    Materials& materials = scene->materials;

    // default pbrt material
    if (!pbrt_material) {
        Lambertian_Material mtl;
        set_constant_parameter(mtl.reflectance, ColorRGB{ 0.5f, 0.5f, 0.5f });
        return add_material<Material_Type::lambertian>(materials, mtl);
    }

    if (auto matte = std::dynamic_pointer_cast<pbrt::MatteMaterial>(pbrt_material)) {
        Lambertian_Material mtl;

        if (matte->map_kd)
            mtl.reflectance = import_pbrt_texture_rgb(matte->map_kd, scene);
        else 
            set_constant_parameter(mtl.reflectance, ColorRGB(&matte->kd.x));

        if (matte->map_bump)
            mtl.bump_map = import_pbrt_texture_float(matte->map_bump, scene);

        return add_material<Material_Type::lambertian>(materials, mtl);
    }

    if (auto translucent_material = std::dynamic_pointer_cast<pbrt::TranslucentMaterial>(pbrt_material)) {
        Diffuse_Transmission_Material mtl;

        if (translucent_material->map_kd)
            mtl.transmittance = import_pbrt_texture_rgb(translucent_material->map_kd, scene);
        else
            set_constant_parameter(mtl.transmittance, ColorRGB(&translucent_material->kd.x));

        set_constant_parameter(mtl.reflectance, ColorRGB(0.25f));
        set_constant_parameter(mtl.scale, 1.f);

        return add_material<Material_Type::diffuse_transmission>(materials, mtl);
    }

    if (auto mirror_material = std::dynamic_pointer_cast<pbrt::MirrorMaterial>(pbrt_material)) {
        Perfect_Reflector_Material mtl;
        set_constant_parameter(mtl.reflectance, ColorRGB(&mirror_material->kr.x));
        return add_material<Material_Type::perfect_reflector>(materials, mtl);
    }

    if (auto glass_material = std::dynamic_pointer_cast<pbrt::GlassMaterial>(pbrt_material)) {
        Glass_Material mtl;
        set_constant_parameter(mtl.reflectance, ColorRGB(&glass_material->kr.x));
        set_constant_parameter(mtl.transmittance, ColorRGB(&glass_material->kt.x));
        set_constant_parameter(mtl.index_of_refraction, glass_material->index);
        return add_material<Material_Type::glass>(materials, mtl);
    }

    if (auto metal = std::dynamic_pointer_cast<pbrt::MetalMaterial>(pbrt_material)) {
        Metal_Material mtl;

        if (metal->map_roughness)
            mtl.roughness = import_pbrt_texture_float(metal->map_roughness, scene);
        else
            set_constant_parameter(mtl.roughness, metal->roughness);
        mtl.roughness_is_alpha = !metal->remapRoughness;

        set_constant_parameter(mtl.eta_i, 1.f);

        if (!metal->spectrum_eta.spd.empty()) {
            Sampled_Spectrum s = to_sampled_spectrum(metal->spectrum_eta);
            Vector3 eta_xyz = s.reflectance_spectrum_to_XYZ_for_D65_illuminant();
            ColorRGB eta_rgb = XYZ_to_sRGB(eta_xyz);
            set_constant_parameter(mtl.eta, eta_rgb);
        }
        else {
            set_constant_parameter(mtl.eta, ColorRGB(&metal->eta.x));
        }

        if (!metal->spectrum_k.spd.empty()) {
            Sampled_Spectrum s = to_sampled_spectrum(metal->spectrum_k);
            Vector3 k_xyz = s.reflectance_spectrum_to_XYZ_for_D65_illuminant();
            ColorRGB k_rgb = XYZ_to_sRGB(k_xyz);
            set_constant_parameter(mtl.k, k_rgb);
        }
        else {
            set_constant_parameter(mtl.k, ColorRGB(&metal->k.x));
        }
        return add_material<Material_Type::metal>(materials, mtl);
    }

    if (auto plastic = std::dynamic_pointer_cast<pbrt::PlasticMaterial>(pbrt_material)) {
        Plastic_Material mtl;

        if (plastic->map_roughness)
            mtl.roughness = import_pbrt_texture_float(plastic->map_roughness, scene);
        else
            set_constant_parameter(mtl.roughness, plastic->roughness);
        mtl.roughness_is_alpha = !plastic->remapRoughness;

        if (plastic->map_ks) {
            mtl.r0 = import_pbrt_texture_float(plastic->map_ks, scene);
        }
        else {
            ColorRGB r0_xyz = sRGB_to_XYZ(ColorRGB(&plastic->ks.x));
            set_constant_parameter(mtl.r0, r0_xyz[1]);
        }

        if (plastic->map_kd)
            mtl.diffuse_reflectance = import_pbrt_texture_rgb(plastic->map_kd, scene);
        else 
            set_constant_parameter(mtl.diffuse_reflectance, ColorRGB(&plastic->kd.x));

        if (plastic->map_bump)
            mtl.bump_map = import_pbrt_texture_float(plastic->map_bump, scene);

        return add_material<Material_Type::plastic>(materials, mtl);
    }

    if (auto coated_diffuse = std::dynamic_pointer_cast<pbrt::SubstrateMaterial>(pbrt_material)) {
        ASSERT(coated_diffuse->map_vRoughness == nullptr);
        ASSERT(coated_diffuse->uRoughness == coated_diffuse->vRoughness);

        Coated_Diffuse_Material mtl;

        if (coated_diffuse->map_uRoughness)
            mtl.roughness = import_pbrt_texture_float(coated_diffuse->map_uRoughness, scene);
        else
            set_constant_parameter(mtl.roughness, coated_diffuse->uRoughness);
        mtl.roughness_is_alpha = !coated_diffuse->remapRoughness;

        if (coated_diffuse->map_ks)
            mtl.r0 = import_pbrt_texture_rgb(coated_diffuse->map_ks, scene);
        else
            set_constant_parameter(mtl.r0, ColorRGB(&coated_diffuse->ks.x));

        if (coated_diffuse->map_kd)
            mtl.diffuse_reflectance = import_pbrt_texture_rgb(coated_diffuse->map_kd, scene);
        else
            set_constant_parameter(mtl.diffuse_reflectance, ColorRGB(&coated_diffuse->kd.x));

        if (coated_diffuse->map_bump)
            mtl.bump_map = import_pbrt_texture_float(coated_diffuse->map_bump, scene);

        return add_material<Material_Type::coated_diffuse>(materials, mtl);
    }

    if (auto uber = std::dynamic_pointer_cast<pbrt::UberMaterial>(pbrt_material)) {
        ASSERT(uber->alpha == 0.f);
        ASSERT(uber->map_alpha == nullptr);
        ASSERT(uber->shadowAlpha == 0.f);
        ASSERT(uber->map_shadowAlpha == nullptr);
        ASSERT(uber->uRoughness == 0.f);
        ASSERT(uber->map_uRoughness == nullptr);
        ASSERT(uber->vRoughness == 0.f);
        ASSERT(uber->map_vRoughness == nullptr);

        Pbrt3_Uber_Material mtl;

        mtl.diffuse_reflectance = init_rgb_parameter_from_texture_or_constant(scene, uber->map_kd, uber->kd);
        if (mtl.diffuse_reflectance.texture_index >= 0 || mtl.diffuse_reflectance.constant_value != Color_Black)
            mtl.components[mtl.component_count++] = Pbrt3_Uber_Material::DIFFUSE;

        mtl.specular_reflectance = init_rgb_parameter_from_texture_or_constant(scene, uber->map_ks, uber->ks);
        if (mtl.specular_reflectance.texture_index >= 0 || mtl.specular_reflectance.constant_value != Color_Black)
            mtl.components[mtl.component_count++] = Pbrt3_Uber_Material::SPECULAR;

        mtl.delta_reflectance = init_rgb_parameter_from_texture_or_constant(scene, uber->map_kr, uber->kr);
        if (mtl.delta_reflectance.texture_index >= 0 || mtl.delta_reflectance.constant_value != Color_Black)
            mtl.components[mtl.component_count++] = Pbrt3_Uber_Material::DELTA_REFLECTION;

        mtl.delta_transmission = init_rgb_parameter_from_texture_or_constant(scene, uber->map_kt, uber->kt);
        if (mtl.delta_transmission.texture_index >= 0 || mtl.delta_transmission.constant_value != Color_Black)
            mtl.components[mtl.component_count++] = Pbrt3_Uber_Material::DELTA_TRANSMISSION;

        mtl.opacity = init_rgb_parameter_from_texture_or_constant(scene, uber->map_opacity, uber->opacity);
        if (mtl.opacity.texture_index >= 0 || mtl.opacity.constant_value != Color_White)
            mtl.components[mtl.component_count++] = Pbrt3_Uber_Material::OPACITY;

        if (uber->map_bump)
            mtl.bump_map = import_pbrt_texture_float(uber->map_bump, scene);

        ASSERT(mtl.component_count <= std::size(mtl.components));

        if (uber->map_roughness)
            mtl.roughness = import_pbrt_texture_float(uber->map_roughness, scene);
        else
            set_constant_parameter(mtl.roughness, uber->roughness);

        // pbrt-parser currently does not support `remaproughness` attribute for uber material (easy to add if necessary).
        // It's not problem in practise - all standard pbrt3 scenes use default remap value (true) for uber material.
        mtl.roughness_is_alpha = false; 

        set_constant_parameter(mtl.index_of_refraction, uber->index);

        return add_material<Material_Type::pbrt3_uber>(materials, mtl);
    }

    // Use red diffuse material to indicate unsupported material.
    Lambertian_Material mtl;
    set_constant_parameter(mtl.reflectance, ColorRGB{ 1.f, 0.f, 0.f });
    return add_material<Material_Type::lambertian>(materials, mtl);
}

static Geometry_Handle import_pbrt_triangle_mesh(const pbrt::TriangleMesh::SP pbrt_mesh, Scene* scene) {
    Triangle_Mesh mesh;

    mesh.indices.resize(pbrt_mesh->index.size() * 3);
    for (auto[i, triangle_indices] : enumerate(pbrt_mesh->index)) {
        mesh.indices[i * 3 + 0] = triangle_indices.x;
        mesh.indices[i * 3 + 1] = triangle_indices.y;
        mesh.indices[i * 3 + 2] = triangle_indices.z;
    }

    mesh.vertices.resize(pbrt_mesh->vertex.size());
    for (size_t i = 0; i < pbrt_mesh->vertex.size(); i++) {
        mesh.vertices[i] = Vector3(&pbrt_mesh->vertex[i].x);
    }

    if (!pbrt_mesh->normal.empty()) {
        ASSERT(pbrt_mesh->normal.size() == pbrt_mesh->vertex.size());
        mesh.normals.reserve(pbrt_mesh->normal.size());
        for (const pbrt::vec3f& normal : pbrt_mesh->normal)
            mesh.normals.push_back(Vector3(&normal.x));
    }

    if (!pbrt_mesh->texcoord.empty()) {
        ASSERT(pbrt_mesh->texcoord.size() == pbrt_mesh->vertex.size());
        mesh.uvs.reserve(pbrt_mesh->texcoord.size());
        for (const pbrt::vec2f& texcoord : pbrt_mesh->texcoord)
            mesh.uvs.push_back(Vector2(&texcoord.x));
    }
    else if (mesh.vertices.size() == 4) {
        // TODO: one improvement might be to use these default values only
        // if mesh material uses parameterization, for example, if texture
        // is used. How to determine this in a simple and robust way?
        mesh.uvs = {
            Vector2(0, 0),
            Vector2(1, 0),
            Vector2(1, 1),
            Vector2(1, 0) // not a typo, follows pbrt defaults
        };
    }

    mesh.reverse_geometric_normal_orientation = pbrt_mesh->reverseOrientation;

    mesh.remove_degenerate_triangles();
    if (mesh.indices.empty())
        return Null_Geometry;

    if (auto alpha_texture_it = pbrt_mesh->textures.find("alpha");
        alpha_texture_it != pbrt_mesh->textures.end())
    {
        auto alpha_texture = std::dynamic_pointer_cast<pbrt::ImageTexture>(alpha_texture_it->second);
        if (alpha_texture)
            mesh.alpha_texture_index = add_scene_texture(alpha_texture->fileName, scene);
    }

    scene->geometries.triangle_meshes.emplace_back(mesh);
    return Geometry_Handle{ Geometry_Type::triangle_mesh, (int)scene->geometries.triangle_meshes.size() - 1 };
}

static Geometry_Handle import_pbrt_sphere(const pbrt::Sphere::SP pbrt_sphere, Matrix3x4* sphere_transform, Scene* scene) {
    Triangle_Mesh sphere = create_sphere_mesh(pbrt_sphere->radius, 6, true);
    scene->geometries.triangle_meshes.push_back(std::move(sphere));
    *sphere_transform = to_matrix3x4(pbrt_sphere->transform);
    return Geometry_Handle{ Geometry_Type::triangle_mesh, int(scene->geometries.triangle_meshes.size() - 1) };
}

static Shape import_pbrt_shape(pbrt::Shape::SP pbrt_shape, const Matrix3x4& instance_transform, Scene* scene) {
    Shape shape;
    if (pbrt::TriangleMesh::SP pbrt_mesh = std::dynamic_pointer_cast<pbrt::TriangleMesh>(pbrt_shape))
    {
        shape.geometry = import_pbrt_triangle_mesh(pbrt_mesh, scene);
        if (shape.geometry == Null_Geometry)
            return Shape{};

        if (pbrt_shape->areaLight != nullptr) {
            if (auto pbrt_diffuse_area_light_rgb = std::dynamic_pointer_cast<pbrt::DiffuseAreaLightRGB>(pbrt_shape->areaLight))
            {
                Vector2 rect_size;
                Matrix3x4 rect_transform;
                const Triangle_Mesh& mesh = scene->geometries.triangle_meshes[shape.geometry.index];
                if (check_if_mesh_is_rectangle(mesh, rect_size, rect_transform)) {
                    if (pbrt_shape->reverseOrientation) {
                        rect_transform.set_column(0, -rect_transform.get_column(0));
                        rect_transform.set_column(2, -rect_transform.get_column(2));
                    }
                    Diffuse_Rectangular_Light light;
                    light.light_to_world_transform = rect_transform;
                    light.emitted_radiance = ColorRGB(&pbrt_diffuse_area_light_rgb->L.x);
                    light.size = rect_size;
                    light.sample_count = pbrt_diffuse_area_light_rgb->nSamples;
                    scene->lights.diffuse_rectangular_lights.push_back(light);
                    shape.area_light = {Light_Type::diffuse_rectangular, (int)scene->lights.diffuse_rectangular_lights.size() - 1};
                }
                else
                    error("triangle mesh light sources are not supported yet");
            }
            else
                error("unsupported area light type");
        }
    }

    if (pbrt::Sphere::SP pbrt_sphere = std::dynamic_pointer_cast<pbrt::Sphere>(pbrt_shape))
    {
        shape.geometry = import_pbrt_sphere(pbrt_sphere, &shape.transform, scene);

        if (pbrt_shape->areaLight != nullptr) {
            if (auto pbrt_diffuse_area_light_rgb = std::dynamic_pointer_cast<pbrt::DiffuseAreaLightRGB>(pbrt_shape->areaLight))
            {
                Diffuse_Sphere_Light light;
                light.position = (instance_transform * shape.transform).get_column(3);
                light.emitted_radiance = ColorRGB(&pbrt_diffuse_area_light_rgb->L.x);
                light.radius = pbrt_sphere->radius;
                light.sample_count = pbrt_diffuse_area_light_rgb->nSamples;
                scene->lights.diffuse_sphere_lights.push_back(light);
                shape.area_light = {Light_Type::diffuse_sphere, (int)scene->lights.diffuse_sphere_lights.size() - 1};
            }
            else
                error("unsupported area light type");
        }
    }

    if (shape.geometry == Null_Geometry)
        error("unsupported pbrt shape type");

    // The covention that area lights only emit light and do not exhibit relfection properties.
    // Here we parse material only if the shape does not have associated area light.
    if (pbrt_shape->areaLight == nullptr)
        shape.material = import_pbrt_material(pbrt_shape->material, scene);

    return shape;
}

static void import_pbrt_non_area_light(pbrt::LightSource::SP pbrt_light, const Matrix3x4& instance_transfrom, Scene* scene)
{
    if (auto point_light = std::dynamic_pointer_cast<pbrt::PointLightSource>(pbrt_light);
        point_light)
    {
        ASSERT(point_light->Ispectrum.spd.empty()); // not supported yet
        Point_Light light;
        light.position = Vector3(&point_light->from.x);
        light.intensity = ColorRGB(&point_light->I.x) * ColorRGB(&point_light->scale.x);
        scene->lights.point_lights.push_back(light);
        return;
    }

    if (auto spot_light = std::dynamic_pointer_cast<pbrt::SpotLightSource>(pbrt_light); 
        spot_light)
    {
        ASSERT(spot_light->Ispectrum.spd.empty()); // not supported yet
        Spot_Light light;
        light.position = Vector3(&spot_light->from.x);
        light.direction = (Vector3(&spot_light->to.x) - Vector3(&spot_light->from.x)).normalized();
        light.cone_angle = radians(spot_light->coneAngle);
        light.penumbra_angle = radians(spot_light->coneDeltaAngle);
        light.intensity = ColorRGB(&spot_light->I.x) * ColorRGB(&spot_light->scale.x);
        scene->lights.spot_lights.push_back(light);
        return;
    }

    if (auto distant_light = std::dynamic_pointer_cast<pbrt::DistantLightSource>(pbrt_light);
        distant_light)
    {
        Vector3 light_vec = Vector3(&distant_light->from.x) - Vector3(&distant_light->to.x);
        light_vec = transform_vector(instance_transfrom, light_vec);

        Directional_Light light;
        light.direction = light_vec.normalized();
        light.irradiance = ColorRGB(&distant_light->L.x) * ColorRGB(&distant_light->scale.x);
        scene->lights.directional_lights.push_back(light);
        return;
    }

    if (auto infinite_light = std::dynamic_pointer_cast<pbrt::InfiniteLightSource>(pbrt_light);
        infinite_light)
    {
        Environment_Light& light = scene->lights.environment_light;
        light.light_to_world = instance_transfrom * to_matrix3x4(infinite_light->transform);
        light.world_to_light = get_inverse_transform(light.light_to_world);
        light.scale = ColorRGB(&infinite_light->scale.x) * ColorRGB(&infinite_light->L.x);

        if (!infinite_light->mapName.empty()) {
            light.environment_map_index = add_scene_texture(infinite_light->mapName, scene);
        }
        else {
            Texture_Descriptor texture_desc{
                .is_constant_texture = true,
                .constant_value = Color_White
            };
            light.environment_map_index = add_scene_texture(texture_desc, scene);
        }

        light.sample_count = infinite_light->nSamples;
        scene->lights.has_environment_light = true;
        return;
    }

    ASSERT(false); // unsupported light type
}

static void import_pbrt_camera(pbrt::Camera::SP pbrt_camera, Scene* scene) {
    const pbrt::math::vec3f& pos = pbrt_camera->frame.p;
    const pbrt::math::mat3f& rot = pbrt_camera->frame.l;

    Matrix3x4 view_point;
    {
        view_point.set_column(3, Vector3(&pos.x));

        // Camera orientation in pbrt's left-handed coordinate system.
        Vector3 right = Vector3(&rot.vx.x);
        Vector3 up = Vector3(&rot.vy.x);
        Vector3 forward = Vector3(&rot.vz.x);

        scene->z_is_up = std::abs(up.z) > std::abs(up.y);

        // Setup camera in right-handed coordinate system according to conventions from "camera.h".
        //
        // This type of code might be non-trivial to understand just by eyeballing it. One way to
        // validate how right/up/forward directions from left-handed CS can be used to construct
        // camera basis in right-handed CS is to draw left-handed coordinate system with a reference
        // object and then check how right/up/forward vectors should be used in right-handed CS to
        // get the same view with the only exception that it will be flipped horizontally
        // (due to different handedness).
        //
        // We don't setup camera in a way that ensures that final image is not flipped horizontally
        // comparing to pbrt output - that's expected behavior that different CS handedness produces
        // mirrored image. It worth to note that it's possible to construct camera basis that
        // will mirror the image (so it will match pbrt) but that's quite confusing during development
        // because of unintuitive relationship between object coordinates and its image plane positioning.
        //
        // If there is a need to have the same output as pbrt then there is a --flip command line
        // option. Another solution is to flip the image by external tool.
        if (scene->z_is_up) {
            view_point.set_column(0, -right);
            view_point.set_column(1, forward);
            view_point.set_column(2, up);
        }
        else { // y_is_up
            view_point.set_column(0, -right);
            view_point.set_column(1, up);
            view_point.set_column(2, -forward);
        }
    }

    if (is_transform_changes_handedness(view_point))
        scene->front_face_has_clockwise_winding = true;

    scene->view_points.push_back(view_point);

    // "fov" in pbrt project files specifies field of view of the more
    // narrow image dimension. For "horizontal" images this represents 
    // vertical field of view which matches our convention (fov_y) but
    // for "vertical" images the pbrt's fov represents horizontal fov,
    // which should be converted to our convention (fov_y).
    if (scene->film_resolution.y > scene->film_resolution.x) {
        float fov_x_over_2_tan = std::tan(radians(pbrt_camera->fov / 2.f));
        float fov_y_over_2_tan = fov_x_over_2_tan * float(scene->film_resolution.y) / float(scene->film_resolution.x);
        scene->camera_fov_y = degrees(2.f * std::atan(fov_y_over_2_tan));
    }
    else {
        scene->camera_fov_y = pbrt_camera->fov;
    }
}

//
// PBRT scene main loading routine.
//
void load_pbrt_scene(const YAR_Project& project, Scene& scene) {
    pbrt::Scene::SP pbrt_scene = pbrt::importPBRT(scene.path);
    pbrt_scene->makeSingleLevel();

    // TODO: re-work pbrt-parser to decouple material from shape to be able to use
    // the same shape with different materials. In current design shape data is
    // duplicated for each new material. pbrt-parser have to introduce primitive
    // abstraction that combines shape and material.

    std::unordered_map<pbrt::Shape::SP, Shape> shape_cache;
    for (pbrt::Instance::SP instance : pbrt_scene->world->instances) {
        ASSERT(instance->object->instances.empty()); // enforced by makeSingleLevel
        Matrix3x4 instance_transform = to_matrix3x4(instance->xfm);

        // Import pbrt shapes.
        for (pbrt::Shape::SP pbrt_shape : instance->object->shapes) {
            Shape shape;
            if (auto it = shape_cache.find(pbrt_shape); it != shape_cache.end()) {
                shape = it->second;
            }
            else {
                shape = import_pbrt_shape(pbrt_shape, instance_transform, &scene);
                shape_cache.insert({pbrt_shape, shape});
            }

            // pbrt shape might not produce a valid geometry (e.g. all triangles are degenerate)
            if (shape.geometry == Null_Geometry)
                continue;

            if (shape.geometry.type == Geometry_Type::triangle_mesh) {
                Scene_Object scene_object;
                scene_object.geometry = shape.geometry;
                scene_object.material = shape.material;
                scene_object.area_light = shape.area_light;
                scene_object.object_to_world_transform = instance_transform * shape.transform;
                scene_object.world_to_object_transform = get_inverse_transform(scene_object.object_to_world_transform);
                scene.objects.push_back(scene_object);

                // Material and area light are mutually exclusive properties and one of them must be defined.
                ASSERT((scene_object.area_light == Null_Light) != (scene_object.material == Null_Material));
            }
        }

        // Import pbrt non-area lights.
        for (pbrt::LightSource::SP light : instance->object->lightSources) {
            import_pbrt_non_area_light(light, instance_transform, &scene);
        }
    }

    // Import film.
    pbrt::Film::SP pbrt_film = pbrt_scene->film;
    if (pbrt_film) {
        scene.output_filename = pbrt_film->fileName;
        scene.film_resolution.x = pbrt_film->resolution.x;
        scene.film_resolution.y = pbrt_film->resolution.y;

        // Initialize render region.
        {
            // Invert computations for x axis to take into account that we use right-handed CS versus left-handed in pbrt.
            int render_region_x0 = scene.film_resolution.x - pbrt_film->cropWindow.z;
            int render_region_x1 = scene.film_resolution.x - pbrt_film->cropWindow.x;

            int render_region_y0 = pbrt_film->cropWindow.y;
            int render_region_y1 = pbrt_film->cropWindow.w;

            scene.render_region.p0 = Vector2i{ render_region_x0, render_region_y0 };
            scene.render_region.p1 = Vector2i{ render_region_x1, render_region_y1 };
        }

        scene.raytracer_config.film_radiance_scale = pbrt_film->scale;
        scene.raytracer_config.max_rgb_component_value_of_film_sample = pbrt_film->maxComponentValue;
    }

    // Import camera.
    ASSERT(!pbrt_scene->cameras.empty());
    pbrt::Camera::SP pbrt_camera = pbrt_scene->cameras[0];
    import_pbrt_camera(pbrt_camera, &scene);

    // Import sampler.
    pbrt::Sampler::SP pbrt_sampler = pbrt_scene->sampler;
    if (pbrt_sampler) {
        scene.raytracer_config.x_pixel_sample_count = pbrt_sampler->xSamples;
        scene.raytracer_config.y_pixel_sample_count = pbrt_sampler->ySamples;
    }

    // Import integrator.
    pbrt::Integrator::SP pbrt_integrator = pbrt_scene->integrator;
    if (pbrt_integrator) {
        if (pbrt_integrator->type == pbrt::Integrator::Type::direct_lighting) {
            scene.raytracer_config.rendering_algorithm = Raytracer_Config::Rendering_Algorithm::direct_lighting;
        }
        else if (pbrt_integrator->type == pbrt::Integrator::Type::path_tracer) {
            scene.raytracer_config.rendering_algorithm = Raytracer_Config::Rendering_Algorithm::path_tracer;
        }
        else if (pbrt_integrator->type == pbrt::Integrator::Type::bidirectional_path_tracer) {
            // TODO: implement BDPT integrator
            scene.raytracer_config.rendering_algorithm = Raytracer_Config::Rendering_Algorithm::path_tracer;
        }
        else {
            error("Unsupported pbrt integrator");
        }

        if (pbrt_integrator->maxDepth >= 0) {
            // In pbrt maxdepth denotes the max number of bounces.
            scene.raytracer_config.max_light_bounces = pbrt_integrator->maxDepth;
        }
        scene.raytracer_config.russian_roulette_threshold = pbrt_integrator->russianRouletteThreshold;
    }

    // Import pixel filter.
    pbrt::PixelFilter::SP pbrt_pixel_filter = pbrt_scene->pixelFilter;
    if (pbrt_pixel_filter) {
        if (pbrt_pixel_filter->type == pbrt::PixelFilter::Type::box) {
            scene.raytracer_config.pixel_filter_type = Raytracer_Config::Pixel_Filter_Type::box;
        }
        else if (pbrt_pixel_filter->type == pbrt::PixelFilter::Type::gaussian) {
            scene.raytracer_config.pixel_filter_type = Raytracer_Config::Pixel_Filter_Type::gaussian;
        }
        else if (pbrt_pixel_filter->type == pbrt::PixelFilter::Type::triangle) {
            scene.raytracer_config.pixel_filter_type = Raytracer_Config::Pixel_Filter_Type::triangle;
        }
        else {
            error("Unsupported pbrt pixel filter");
        }
        scene.raytracer_config.pixel_filter_radius = pbrt_pixel_filter->radius;
        scene.raytracer_config.pixel_filter_alpha = pbrt_pixel_filter->alpha;
    }
}
