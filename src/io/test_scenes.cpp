#include "test_scenes.h"
#include "reference_cpu/colorimetry.h"
#include "reference_cpu/spectrum.h"

Scene_Data load_conference_scene() {
    Matrix3x4 to_world_axis_orientation {{
        {1, 0,  0, 0},
        {0, 0, -1, 0},
        {0, 1,  0, 0}
    }};

    // Uniform spectrum that produces luminous flux of 1600Lm.
    float P = 1600; // Lm
    float C = P / (683.f * CIE_Y_integral); // [W/m]
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(C);
    XYZ xyz = s.emission_spectrum_to_XYZ();

    RGB_Point_Light_Data light;
    light.position = Vector3(0, 0, 1.5);
    light.intensity = RGB(xyz);

    Scene_Data scene;
    scene.meshes = load_obj("conference/conference.obj", uniform_scale(to_world_axis_orientation, 0.003f));
    scene.rgb_point_lights.push_back(light);
    return scene;
}
