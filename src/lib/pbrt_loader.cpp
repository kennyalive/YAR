#include "std.h"
#include "common.h"
#include "pbrt_loader.h"

#include "pbrt-parser/pbrt_parser.h"

#include "colorimetry.h"
static ColorRGB convert_flux_to_constant_spectrum_to_rgb_intensity(float luminous_flux) {
    float radiant_flux_per_wavelength = luminous_flux / (683.f * CIE_Y_integral); // [W/m]
                                                                                  // Get constant spectrum that produces given luminous_flux.
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(radiant_flux_per_wavelength);
    Vector3 xyz = s.emission_spectrum_to_XYZ();
    // Constant spectrum does not produce white RGB (for sRGB). It's a bit reddish.
    return ColorRGBFromXYZ(xyz);
}

Scene_Data load_pbrt_scene(const YAR_Project& project) {
    using  namespace pbrt::semantic;
    std::shared_ptr<Scene> scene = importPBRT(project.scene_path);

    Scene_Data scene_data;
    scene_data.project_dir = "pbrt-dragon"; // TODO: temporarily hardcoded

    for (Shape::SP geom : scene->world->shapes) {
        if (TriangleMesh::SP triangle_mesh = std::dynamic_pointer_cast<TriangleMesh>(geom); triangle_mesh != nullptr) {
            Mesh_Data mesh_data;

            mesh_data.indices.resize(triangle_mesh->index.size() * 3);
            for (auto [i, triangle_indices] : enumerate(triangle_mesh->index)) {
                mesh_data.indices[i*3 + 0] = triangle_indices.x;
                mesh_data.indices[i*3 + 1] = triangle_indices.y;
                mesh_data.indices[i*3 + 2] = triangle_indices.z;
            }

            bool has_normals = !triangle_mesh->normal.empty();

            mesh_data.vertices.resize(triangle_mesh->vertex.size());
            ASSERT(!has_normals || triangle_mesh->vertex.size() == triangle_mesh->normal.size());
            for (size_t i = 0; i < triangle_mesh->vertex.size(); i++) {
                mesh_data.vertices[i].pos = 0.01f * Vector3(&triangle_mesh->vertex[i].x);
                if (has_normals) {
                    mesh_data.vertices[i].normal = Vector3(&triangle_mesh->normal[i].x);
                }
                mesh_data.vertices[i].uv = Vector2_Zero;
            }

            if (!has_normals) {
                compute_normals(mesh_data, Normal_Average_Mode::area, 0.f);
            }

            scene_data.meshes.emplace_back(mesh_data);

            Material_Data material{};
            material.material_format = Material_Format::obj_material;
            material.obj_material.k_diffuse = ColorRGB{ 0.5f, 0.5f, 0.5f };
            scene_data.materials.push_back(material);
        }
    }

    Matrix3x4 view_point{
        -0.775517f, -0.538342f, -0.329897f, 1.714088f,
        0.631373f, -0.661252f, -0.405215f, 1.813797f,
        0.000000f, -0.522510f, 0.852659f, 1.208899f,
    };
    scene_data.view_points.push_back(view_point);

    RGB_Point_Light_Data light;
    light.position = Vector3(2.f, 2.f, 2.f);
    light.intensity = convert_flux_to_constant_spectrum_to_rgb_intensity(2000 /*Lm*/);
    scene_data.rgb_point_lights.push_back(light);

    return scene_data;
}
