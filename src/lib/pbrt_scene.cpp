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

// here we define disney roughness as quantity such that alpha = roughness^2
static float pbrt_roughness_to_disney_roughness(float pbrt_roughness, bool remap) {
    float alpha = pbrt_roughness;
    if (remap) {
        pbrt_roughness = std::max(pbrt_roughness, 1e-3f);
        float x = std::log(pbrt_roughness);
        alpha = 1.62142f + 0.819955f*x + 0.1734f*x*x + 0.0171201f*x*x*x + 0.000640711f*x*x*x*x;
    }
    float roughness = std::sqrt(alpha);
    return roughness;
}

static bool check_if_mesh_is_rectangle(const Triangle_Mesh& mesh, Vector2& size, Matrix3x4& transform) {
    if (mesh.vertices.size() != 4 || mesh.indices.size() != 6)
        return false;

    Vector3 p[3] = {
        mesh.vertices[mesh.indices[0]],
        mesh.vertices[mesh.indices[1]],
        mesh.vertices[mesh.indices[2]]
    };
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
    if (k == 3)
        return false;

    Vector3 mid_point = (mesh.vertices[0] + mesh.vertices[1] + mesh.vertices[2] + mesh.vertices[3]) * 0.25f;
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
        int texture_index = add_scene_texture(image_texture->fileName, scene);
        set_texture_parameter(param, texture_index);

        param.u_scale = image_texture->uscale;
        param.v_scale = image_texture->vscale;
    }
    else if (auto constant_texture = std::dynamic_pointer_cast<pbrt::ConstantTexture>(pbrt_texture);
        constant_texture != nullptr)
    {
        set_constant_parameter(param, ColorRGB(&constant_texture->value.x));
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
        int texture_index = add_scene_texture(image_texture->fileName, scene);
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
    else {
        static_assert(false, "add_material: Unexpected Material_Type value");
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

static Material_Handle import_pbrt_material(const pbrt::Material::SP pbrt_material, Scene* scene) {
    Materials& materials = scene->materials;

    if (auto matte = std::dynamic_pointer_cast<pbrt::MatteMaterial>(pbrt_material)) {
        Lambertian_Material mtl;

        if (matte->map_kd)
            mtl.reflectance = import_pbrt_texture_rgb(matte->map_kd, scene);
        else 
            set_constant_parameter(mtl.reflectance, ColorRGB(&matte->kd.x));

        return add_material<Material_Type::lambertian>(materials, mtl);
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
        float roughness = pbrt_roughness_to_disney_roughness(metal->roughness, metal->remapRoughness);
        Metal_Material mtl;
        set_constant_parameter(mtl.roughness, roughness);
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
        ASSERT(plastic->map_bump == nullptr);
        ASSERT(plastic->map_roughness == nullptr);

        float roughness = pbrt_roughness_to_disney_roughness(plastic->roughness, plastic->remapRoughness);
        Plastic_Material mtl;
        set_constant_parameter(mtl.roughness, roughness);

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

        return add_material<Material_Type::plastic>(materials, mtl);
    }

    if (auto coated_diffuse = std::dynamic_pointer_cast<pbrt::SubstrateMaterial>(pbrt_material)) {
        ASSERT(coated_diffuse->map_uRoughness == nullptr);
        ASSERT(coated_diffuse->map_vRoughness == nullptr);
        ASSERT(coated_diffuse->map_bump == nullptr);

        ASSERT(coated_diffuse->uRoughness == coated_diffuse->vRoughness);
        float roughness = pbrt_roughness_to_disney_roughness(coated_diffuse->uRoughness, coated_diffuse->remapRoughness);

        Coated_Diffuse_Material mtl;
        set_constant_parameter(mtl.roughness, roughness);

        if (coated_diffuse->map_ks)
            mtl.r0 = import_pbrt_texture_rgb(coated_diffuse->map_ks, scene);
        else
            set_constant_parameter(mtl.r0, ColorRGB(&coated_diffuse->ks.x));

        if (coated_diffuse->map_kd)
            mtl.diffuse_reflectance = import_pbrt_texture_rgb(coated_diffuse->map_kd, scene);
        else
            set_constant_parameter(mtl.diffuse_reflectance, ColorRGB(&coated_diffuse->kd.x));

        return add_material<Material_Type::coated_diffuse>(materials, mtl);
    }

    // Default material.
    Lambertian_Material mtl;
    set_constant_parameter(mtl.reflectance, ColorRGB{ 0.5f, 0.5f, 0.5f });
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

    bool has_normals = !pbrt_mesh->normal.empty();

    mesh.vertices.resize(pbrt_mesh->vertex.size());
    mesh.normals.resize(pbrt_mesh->vertex.size());
    mesh.uvs.resize(pbrt_mesh->vertex.size());
    ASSERT(!has_normals || pbrt_mesh->vertex.size() == pbrt_mesh->normal.size());
    for (size_t i = 0; i < pbrt_mesh->vertex.size(); i++) {
        mesh.vertices[i] = Vector3(&pbrt_mesh->vertex[i].x);

        if (has_normals)
            mesh.normals[i] = Vector3(&pbrt_mesh->normal[i].x);

        if (pbrt_mesh->texcoord.size())
            mesh.uvs[i] = Vector2(&pbrt_mesh->texcoord[i].x);
        else
            mesh.uvs[i] = Vector2_Zero;
    }

    mesh.remove_degenerate_triangles();
    if (mesh.indices.empty())
        return Null_Geometry;

    if (!has_normals) {
        Normal_Calculation_Params normal_params;
        calculate_normals(normal_params, mesh);
    }

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

static void import_pbrt_non_area_light(pbrt::LightSource::SP pbrt_light, const Matrix3x4& instance_transfrom, Scene* scene) {
    auto distant_light = std::dynamic_pointer_cast<pbrt::DistantLightSource>(pbrt_light);
    if (distant_light)
    {
        Vector3 light_vec = Vector3(&distant_light->from.x) - Vector3(&distant_light->to.x);
        light_vec = transform_vector(instance_transfrom, light_vec);

        Directional_Light light;
        light.direction = light_vec.normalized();
        light.irradiance = ColorRGB(&distant_light->L.x) * ColorRGB(&distant_light->scale.x);
        scene->lights.directional_lights.push_back(light);
    }

    auto infinite_light = std::dynamic_pointer_cast<pbrt::InfiniteLightSource>(pbrt_light);
    if (infinite_light) {
        Environment_Light& light = scene->lights.environment_light;
        light.light_to_world = instance_transfrom * to_matrix3x4(infinite_light->transform);
        light.world_to_light = get_inverse_transform(light.light_to_world);
        light.scale = ColorRGB(&infinite_light->scale.x) * ColorRGB(&infinite_light->L.x);
        light.environment_map_index = add_scene_texture(infinite_light->mapName, scene);
        light.sample_count = infinite_light->nSamples;
        scene->lights.has_environment_light = true;
    }
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
    if (scene->image_resolution.y > scene->image_resolution.x) {
        float fov_x_over_2_tan = std::tan(radians(pbrt_camera->fov / 2.f));
        float fov_y_over_2_tan = fov_x_over_2_tan * float(scene->image_resolution.y) / float(scene->image_resolution.x);
        scene->camera_fov_y = degrees(2.f * std::atan(fov_y_over_2_tan));
    }
    else {
        scene->camera_fov_y = pbrt_camera->fov;
    }
}

//
// PBRT scene main loading routine.
//
Scene load_pbrt_scene(const YAR_Project& project) {
    pbrt::Scene::SP pbrt_scene = pbrt::importPBRT(project.scene_path.string());
    pbrt_scene->makeSingleLevel();

    // TODO: re-work pbrt-parser to decouple material from shape to be able to use
    // the same shape with different materials. In current design shape data is
    // duplicated for each new material. pbrt-parser have to introduce primitive
    // abstraction that combines shape and material.

    Scene scene;

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
        scene.image_resolution.x = pbrt_film->resolution.x;
        scene.image_resolution.y = pbrt_film->resolution.y;
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
    return scene;
}
