#include "std.h"
#include "common.h"

#include "colorimetry.h"
#include "spectrum.h"
#include "scene.h"
#include "yar_project.h"

#include "pbrtParser/Scene.h"
#include "pbrt-parser/impl/syntactic/Scene.h"

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

static float pbrt_roughness_to_everyday_people_roughness(float pbrt_roughness) {
    pbrt_roughness = std::max(pbrt_roughness, 1e-3f);
    float x = std::log(pbrt_roughness);
    float alpha = 1.62142f + 0.819955f*x + 0.1734f*x*x + 0.0171201f*x*x*x + 0.000640711f*x*x*x*x;
    float everyday_people_roughness = std::sqrt(alpha);
    return everyday_people_roughness;
}

static Material_Handle import_pbrt_material(const pbrt::Material::SP pbrt_material, Materials* materials) {
    auto matte_material = std::dynamic_pointer_cast<pbrt::MatteMaterial>(pbrt_material);
    if (matte_material) {
        Lambertian_Material mtl;
        bool has_texture = false;
        if (matte_material->map_kd != nullptr) {
            if (pbrt::ImageTexture::SP image_texture = std::dynamic_pointer_cast<pbrt::ImageTexture>(matte_material->map_kd);
                image_texture != nullptr)
            {
                has_texture = true;
                set_constant_parameter(mtl.reflectance, Color_White);
                materials->texture_names.push_back(image_texture->fileName);
                mtl.reflectance.texture_index = (int)materials->texture_names.size() - 1;

                pbrt::syntactic::Texture::SP syntactic_texture = image_texture->syntacticObject;
                mtl.reflectance.u_scale = syntactic_texture->getParam1f("uscale", 1.f);
                mtl.reflectance.v_scale = syntactic_texture->getParam1f("vscale", 1.f);
            }
        }
        if (!has_texture) {
            set_constant_parameter(mtl.reflectance, ColorRGB(&matte_material->kd.x));
        }
        materials->lambertian.push_back(mtl);
        return Material_Handle{ Material_Type::lambertian, int(materials->lambertian.size() - 1) };
    }

    auto mirror_material = std::dynamic_pointer_cast<pbrt::MirrorMaterial>(pbrt_material);
    if (mirror_material) {
        Mirror_Material mtl;
        set_constant_parameter(mtl.reflectance, ColorRGB(&mirror_material->kr.x));
        materials->mirror.push_back(mtl);
        return Material_Handle{ Material_Type::mirror, int(materials->mirror.size() - 1) };
    }

    auto metal = std::dynamic_pointer_cast<pbrt::MetalMaterial>(pbrt_material);
    if (metal) {
        float roughness = pbrt_roughness_to_everyday_people_roughness(metal->roughness);
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
        materials->metal.push_back(mtl);
        return Material_Handle{ Material_Type::metal, int(materials->metal.size() - 1) };
    }

    auto plastic = std::dynamic_pointer_cast<pbrt::PlasticMaterial>(pbrt_material);
    if (plastic) {
        // TODO: handle texture params
        ASSERT(plastic->map_kd == nullptr);
        ASSERT(plastic->map_ks == nullptr);
        ASSERT(plastic->map_bump == nullptr);
        ASSERT(plastic->map_roughness == nullptr);

        float roughness = pbrt_roughness_to_everyday_people_roughness(plastic->roughness);
        Plastic_Material mtl;
        set_constant_parameter(mtl.roughness, roughness);
        ColorRGB r0_xyz = sRGB_to_XYZ(ColorRGB(&plastic->ks.x));
        set_constant_parameter(mtl.r0, r0_xyz[1]);
        set_constant_parameter(mtl.diffuse_reflectance, ColorRGB(&plastic->kd.x));

        materials->plastic.push_back(mtl);
        return Material_Handle{ Material_Type::plastic, int(materials->plastic.size() - 1) };
    }

    // Default material.
    Lambertian_Material mtl;
    set_constant_parameter(mtl.reflectance, ColorRGB{ 0.5f, 0.5f, 0.5f });
    materials->lambertian.push_back(mtl);
    return Material_Handle{ Material_Type::lambertian, int(materials->lambertian.size() - 1) };
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

    if (!has_normals)
        compute_normals(mesh, Normal_Average_Mode::area, 0.f);

    scene->geometries.triangle_meshes.emplace_back(mesh);
    return Geometry_Handle{ Geometry_Type::triangle_mesh, (int)scene->geometries.triangle_meshes.size() - 1 };
}

static Geometry_Handle import_pbrt_sphere(const pbrt::Sphere::SP pbrt_sphere , Scene* scene) {
    Sphere sphere;
    sphere.radius = pbrt_sphere->radius;
    sphere.origin = to_matrix3x4(pbrt_sphere->transform).get_column(3);
    scene->geometries.spheres.push_back(sphere);
    return Geometry_Handle{ Geometry_Type::sphere, int(scene->geometries.spheres.size() - 1) };
}

static void import_pbrt_camera(pbrt::Camera::SP pbrt_camera, Scene* scene) {
    const pbrt::math::vec3f& pos = pbrt_camera->frame.p;
    const pbrt::math::mat3f& rot = pbrt_camera->frame.l;

    Matrix3x4 view_point;
    {
        view_point.set_column(3, Vector3(&pos.x));

        // Assuming left-handed coordinate system (pbrt convention), the pbrt-parser library
        // does the following setup of the camera orientation vectors:
        // rot.vx - right vector
        // rot.vy - up vector
        // rot.vz - forward vector
        Vector3 right = Vector3(&rot.vx.x);
        Vector3 up = Vector3(&rot.vy.x);
        Vector3 forward = Vector3(&rot.vz.x);

        scene->z_is_up = std::abs(up.z) > std::abs(up.y);

        view_point.set_column(0, right);
        if (scene->z_is_up) {
            view_point.set_column(2, up);
            Vector3 y_axis = -forward;
            // By inverting y_axis we effectively generate rays in the opposite direction
            // along y axis so the result matches PBRT output which uses LH convention.
            view_point.set_column(1, -y_axis);
        }
        else { // y_is_up
            view_point.set_column(1, up);
            Vector3 z_axis = forward;
            // By inverting z_axis we effectively generate rays in the opposite direction
            // along z axis so the result matches PBRT output which uses LH convention.
            view_point.set_column(2, -z_axis);
        }
    }

    if (is_transform_changes_handedness(view_point))
        scene->front_face_has_clockwise_winding = true;

    scene->view_points.push_back(view_point);
    scene->camera_fov_y = pbrt_camera->fov;
}

Scene load_pbrt_scene(const YAR_Project& project) {
    pbrt::Scene::SP pbrt_scene = pbrt::importPBRT(project.scene_path.string());
    pbrt_scene->makeSingleLevel();

    struct Imported_Shape {
        Geometry_Handle geometry;
        Material_Handle material;
    };
    std::unordered_map<pbrt::Shape::SP, Imported_Shape> already_imported_shapes;

    Scene scene;

    for (pbrt::Instance::SP instance : pbrt_scene->world->instances) {
        ASSERT(instance->object->instances.empty()); // we flattened instance hierarchy

        // Import pbrt shapes.
        for (pbrt::Shape::SP shape : instance->object->shapes) {
            
            Imported_Shape imported_shape = already_imported_shapes[shape];

            if (imported_shape.geometry == Null_Geometry) {
                pbrt::TriangleMesh::SP pbrt_mesh = std::dynamic_pointer_cast<pbrt::TriangleMesh>(shape);
                if (pbrt_mesh != nullptr)
                    imported_shape.geometry = import_pbrt_triangle_mesh(pbrt_mesh, &scene);

                pbrt::Sphere::SP pbrt_sphere = std::dynamic_pointer_cast<pbrt::Sphere>(shape);
                if (pbrt_sphere != nullptr)
                    imported_shape.geometry = import_pbrt_sphere(pbrt_sphere, &scene);

                if (imported_shape.geometry == Null_Geometry)
                    error("Unsupported pbrt shape type");

                imported_shape.material = import_pbrt_material(shape->material, &scene.materials);
                already_imported_shapes.insert(std::make_pair(shape, imported_shape));
            }

            if (imported_shape.geometry.type == Geometry_Type::triangle_mesh) {
                Scene_Object scene_object;
                scene_object.geometry = imported_shape.geometry;
                scene_object.material = imported_shape.material;
                scene_object.object_to_world_transform = to_matrix3x4(instance->xfm);
                scene_object.world_to_object_transform = get_inverted_transform(scene_object.object_to_world_transform);
                scene.objects.push_back(scene_object);
            }
        }

        // Import pbrt non-area lights.
        for (pbrt::LightSource::SP light : instance->object->lightSources) {
            if (pbrt::DistantLightSource::SP distant_light = std::dynamic_pointer_cast<pbrt::DistantLightSource>(light);
                distant_light != nullptr)
            {
                Directional_Light light;
                Vector3 light_vec = Vector3(&distant_light->from.x) - Vector3(&distant_light->to.x);
                light_vec = transform_vector(to_matrix3x4(instance->xfm), light_vec);
                light.direction = light_vec.normalized();
                light.irradiance = ColorRGB(&distant_light->L.x) * ColorRGB(&distant_light->scale.x);
                scene.lights.directional_lights.push_back(light);
            }
        }
    }

    // Import camera.
    ASSERT(!pbrt_scene->cameras.empty());
    pbrt::Camera::SP pbrt_camera = pbrt_scene->cameras[0];
    import_pbrt_camera(pbrt_camera, &scene);

    // Import film.
    pbrt::Film::SP pbrt_film = pbrt_scene->film;
    if (pbrt_film) {
        scene.image_resolution.x = pbrt_film->resolution.x;
        scene.image_resolution.y = pbrt_film->resolution.y;
    }

    scene.lights.append(project.lights);
    return scene;
}
