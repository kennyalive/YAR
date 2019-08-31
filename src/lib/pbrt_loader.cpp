#include "std.h"
#include "common.h"
#include "pbrt_loader.h"

#include "colorimetry.h"
#include "project.h"
#include "render_object.h"

#include "pbrtParser/Scene.h"

static ColorRGB convert_flux_to_constant_spectrum_to_rgb_intensity(float luminous_flux) {
    float radiant_flux_per_wavelength = luminous_flux / (683.f * CIE_Y_integral); // [W/m]
                                                                                  // Get constant spectrum that produces given luminous_flux.
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(radiant_flux_per_wavelength);
    Vector3 xyz = s.emission_spectrum_to_XYZ();
    // Constant spectrum does not produce white RGB (for sRGB). It's a bit reddish.
    return ColorRGBFromXYZ(xyz);
}

Scene load_pbrt_project(const YAR_Project& project) {
    std::shared_ptr<pbrt::Scene> pbrt_scene = pbrt::importPBRT(project.scene_path);
    Scene scene;

    for (pbrt::Shape::SP geom : pbrt_scene->world->shapes) {
        if (pbrt::TriangleMesh::SP triangle_mesh = std::dynamic_pointer_cast<pbrt::TriangleMesh>(geom); triangle_mesh != nullptr) {
            Triangle_Mesh mesh;

            mesh.indices.resize(triangle_mesh->index.size() * 3);
            for (auto [i, triangle_indices] : enumerate(triangle_mesh->index)) {
                mesh.indices[i*3 + 0] = triangle_indices.x;
                // change winding order (z <-> y) because we will flip geometry
                // to get geometry data in right-handed coordinate system.
                mesh.indices[i*3 + 1] = triangle_indices.z;
                mesh.indices[i*3 + 2] = triangle_indices.y;
            }

            bool has_normals = !triangle_mesh->normal.empty();

            mesh.vertices.resize(triangle_mesh->vertex.size());
            mesh.normals.resize(triangle_mesh->vertex.size());
            mesh.uvs.resize(triangle_mesh->vertex.size());
            ASSERT(!has_normals || triangle_mesh->vertex.size() == triangle_mesh->normal.size());
            for (size_t i = 0; i < triangle_mesh->vertex.size(); i++) {
                mesh.vertices[i] = Vector3(&triangle_mesh->vertex[i].x);

                // Flip geometry around x axis to get coordinates in right-handed coordinate system.
                mesh.vertices[i].x = -mesh.vertices[i].x;

                if (has_normals) 
                    mesh.normals[i] = Vector3(&triangle_mesh->normal[i].x);

                mesh.uvs[i] = Vector2_Zero;
            }

            if (!has_normals)
                compute_normals(mesh, Normal_Average_Mode::area, 0.f);

            scene.geometries.triangle_meshes.emplace_back(mesh);

            Lambertian_Material mtl;
            mtl.albedo = ColorRGB{ 0.5f, 0.5f, 0.5f };
            scene.materials.lambertian.push_back(mtl);

            Render_Object render_object;
            render_object.geometry = {Geometry_Type::triangle_mesh, (int)scene.geometries.triangle_meshes.size() - 1};
            render_object.material = {Material_Type::lambertian, (int)scene.materials.lambertian.size() - 1};
            render_object.object_to_world_transform = Matrix3x4::identity;
            render_object.world_to_object_transform = Matrix3x4::identity;
            scene.render_objects.push_back(render_object);
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
        view_point = get_mirrored_transform(view_point, 0);
    }

    scene.view_points.push_back(view_point);
    scene.fovy = pbrt_scene->cameras[0]->fov;

    Point_Light light;
    light.position = Vector3(-pos.x, pos.y, pos.z);
    light.intensity = convert_flux_to_constant_spectrum_to_rgb_intensity(8000000 /*Lm*/);
    scene.lights.point_lights.push_back(light);

    return scene;
}
