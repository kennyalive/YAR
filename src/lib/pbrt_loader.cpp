#include "std.h"
#include "common.h"
#include "pbrt_loader.h"

#include "colorimetry.h"
#include "project.h"
#include "scene_object.h"

#include "pbrtParser/Scene.h"

static ColorRGB convert_flux_to_constant_spectrum_to_rgb_intensity(float luminous_flux) {
    float radiant_flux_per_wavelength = luminous_flux / (683.f * CIE_Y_integral); // [W/m]
                                                                                  // Get constant spectrum that produces given luminous_flux.
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(radiant_flux_per_wavelength);
    Vector3 xyz = s.emission_spectrum_to_XYZ();
    // Constant spectrum does not produce white RGB (for sRGB). It's a bit reddish.
    return ColorRGBFromXYZ(xyz);
}

// Returns objects-to-world transform.
static Matrix3x4 get_transform_from_pbrt_transform(const pbrt::affine3f& pbrt_transform) {
    const pbrt::math::vec3f& pos = pbrt_transform.p;
    const pbrt::math::mat3f& rot = pbrt_transform.l;
    Matrix3x4 transform;
    transform.set_column(0, Vector3(&rot.vx.x));
    transform.set_column(1, Vector3(&rot.vy.x));
    transform.set_column(2, Vector3(&rot.vz.x));
    transform.set_column(3, Vector3(&pos.x));
    return transform;
}

Scene load_pbrt_project(const YAR_Project& project) {
    std::shared_ptr<pbrt::Scene> pbrt_scene = pbrt::importPBRT(project.scene_path);
    pbrt_scene->makeSingleLevel();
    Scene scene;

    std::map<pbrt::Shape::SP, Geometry_Handle> processed_shapes;

    auto process_shape = [&scene, &processed_shapes](pbrt::Shape::SP shape) {
        if (pbrt::TriangleMesh::SP triangle_mesh = std::dynamic_pointer_cast<pbrt::TriangleMesh>(shape); triangle_mesh != nullptr) {
            Triangle_Mesh mesh;

            mesh.indices.resize(triangle_mesh->index.size() * 3);
            for (auto [i, triangle_indices] : enumerate(triangle_mesh->index)) {
                mesh.indices[i*3 + 0] = triangle_indices.x;
                mesh.indices[i*3 + 1] = triangle_indices.y;
                mesh.indices[i*3 + 2] = triangle_indices.z;
            }

            bool has_normals = !triangle_mesh->normal.empty();

            mesh.vertices.resize(triangle_mesh->vertex.size());
            mesh.normals.resize(triangle_mesh->vertex.size());
            mesh.uvs.resize(triangle_mesh->vertex.size());
            ASSERT(!has_normals || triangle_mesh->vertex.size() == triangle_mesh->normal.size());
            for (size_t i = 0; i < triangle_mesh->vertex.size(); i++) {
                mesh.vertices[i] = Vector3(&triangle_mesh->vertex[i].x);

                if (has_normals) 
                    mesh.normals[i] = Vector3(&triangle_mesh->normal[i].x);

                mesh.uvs[i] = Vector2_Zero;
            }

            if (!has_normals)
                compute_normals(mesh, Normal_Average_Mode::area, 0.f);

            scene.geometries.triangle_meshes.emplace_back(mesh);

            Lambertian_Material mtl;
            mtl.reflectance.is_constant = true;
            mtl.reflectance.constant_value = ColorRGB{ 0.5f, 0.5f, 0.5f };
            scene.materials.lambertian.push_back(mtl);

            Geometry_Handle geometry = Geometry_Handle{ Geometry_Type::triangle_mesh, (int)scene.geometries.triangle_meshes.size() - 1 };
            processed_shapes.insert(std::make_pair(shape, geometry));
            return geometry;
        }
        else {
            error("Unsupported pbrt shape type");
            return Null_Geometry;
        }
    };

    for (pbrt::Instance::SP instance : pbrt_scene->world->instances) {
        ASSERT(instance->object->instances.empty());

        // Process instance's shapes
        for (pbrt::Shape::SP shape : instance->object->shapes) {
            Geometry_Handle geometry;
            auto shape_it = processed_shapes.find(shape);
            if (shape_it != processed_shapes.end())
                geometry = shape_it->second;
            else
                geometry = process_shape(shape);

            Scene_Object scene_object;
            scene_object.geometry = geometry; 
            scene_object.material = {Material_Type::lambertian, (int)scene.materials.lambertian.size() - 1};
            scene_object.object_to_world_transform = get_transform_from_pbrt_transform(instance->xfm);
            scene_object.world_to_object_transform = get_inverted_transform(scene_object.object_to_world_transform);
            scene.objects.push_back(scene_object);
        }

        // Process instance's non-area lights
        for (pbrt::LightSource::SP light : instance->object->lightSources) {
            if (pbrt::DistantLightSource::SP distant_light = std::dynamic_pointer_cast<pbrt::DistantLightSource>(light);
                distant_light != nullptr)
            {
                Directional_Light light;
                Vector3 light_vec = Vector3(&distant_light->from.x) - Vector3(&distant_light->to.x);
                light_vec = transform_vector(get_transform_from_pbrt_transform(instance->xfm), light_vec);
                light.direction = light_vec.normalized();
                light.irradiance = ColorRGB(&distant_light->L.x) * ColorRGB(&distant_light->scale.x);
                scene.lights.directional_lights.push_back(light);
            }
        }
    }

    ASSERT(!pbrt_scene->cameras.empty());
    pbrt::Camera::SP pbrt_camera = pbrt_scene->cameras[0];
    const pbrt::math::vec3f& pos = pbrt_camera->frame.p;
    const pbrt::math::mat3f& rot = pbrt_camera->frame.l;

    Matrix3x4 view_point;
    {
        view_point.set_column(0, Vector3(&rot.vx.x));
        view_point.set_column(1, Vector3(&rot.vz.x));
        view_point.set_column(2, Vector3(&rot.vy.x));
        view_point.set_column(3, Vector3(&pos.x));
    }

    if (is_transform_changes_handedness(view_point))
        scene.front_face_has_clockwise_winding = true;

    scene.view_points.push_back(view_point);
    scene.fovy = pbrt_scene->cameras[0]->fov;
    scene.lights.append(project.lights);
    
    return scene;
}

