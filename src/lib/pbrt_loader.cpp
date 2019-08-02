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
    std::shared_ptr<pbrt::Scene> pbrt_scene = pbrt::importPBRT(project.path);

    ::Scene scene;
    scene.project_dir = "pbrt-dragon"; // TODO: temporarily hardcoded

    for (pbrt::Shape::SP geom : pbrt_scene->world->shapes) {
        if (pbrt::TriangleMesh::SP triangle_mesh = std::dynamic_pointer_cast<pbrt::TriangleMesh>(geom); triangle_mesh != nullptr) {
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
                mesh.vertices[i] = 0.01f * Vector3(&triangle_mesh->vertex[i].x);
                if (has_normals) {
                    mesh.normals[i] = Vector3(&triangle_mesh->normal[i].x);
                }
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

    Matrix3x4 view_point{
        -0.775517f, -0.538342f, -0.329897f, 1.714088f,
        0.631373f, -0.661252f, -0.405215f, 1.813797f,
        0.000000f, -0.522510f, 0.852659f, 1.208899f,
    };
    scene.view_points.push_back(view_point);

    Point_Light light;
    light.position = Vector3(2.f, 2.f, 2.f);
    light.intensity = convert_flux_to_constant_spectrum_to_rgb_intensity(2000 /*Lm*/);
    scene.lights.point_lights.push_back(light);

    return scene;
}
