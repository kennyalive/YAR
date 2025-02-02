#include "std.h"
#include "lib/common.h"

#include "pbrt-parser/include/pbrtParser/Scene.h"

const std::string pbrt3_scenes_directory = "h:/pbrt/pbrt-v3-scenes/";

const std::array benchmark_scenes = {
    "ecosys/ecosys.pbrt",
    "transparent-machines/frame675.pbrt",
    "sportscar/sportscar.pbrt",
    "sanmiguel/sanmiguel.pbrt",
    "landscape/view-0.pbrt",
    "hair/straight-hair.pbrt",
};

void benchmark_pbrt_parser()
{
    printf("-------------\n");
    printf("Benchmark: pbrt-parser\n");
    for (const char* scene_relative_paths : benchmark_scenes) {
        const std::string scene_path = pbrt3_scenes_directory + scene_relative_paths;
        printf("scene       : %s\n", scene_relative_paths);
        Timestamp t;

        pbrt::Scene::SP pbrt_scene = pbrt::importPBRT(scene_path);

        int64_t ns = elapsed_nanoseconds(t);
        double seconds = ns / 1'000'000'000.0;
        printf("parsing time: %.3f seconds\n", seconds);
    }
}
