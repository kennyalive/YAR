# YAR

This is a C++ program for rendering 3D scenes. I use it to experiment with various programming ideas and to practice. While the source code is available and licensed accordingly, the program is not really an open-source project in the modern sense of a collaborative software development process.

The main effort was put into the reference software renderer. It is the `RAY` project in the Visual Studio solution. The source code is in the `src/reference` directory.

There is also an embryonic GPU renderer in the `src/renderer` folder. It is exposed as the `YAR` project in the Visual Studio solution. I did not put much effort into it.

The reference renderer supports a subset of the [pbrt 3 file format](https://pbrt.org/fileformat-v3). It supports scenes that describe the 3D world as triangle meshes. It does not support volumetric rendering or subsurface scattering. Only unidirectional path tracing is supported.

Some areas of focus:
* Deterministic rendering
* Rendering regression testing (I have accompanying repos with rendering snapshots)
* Investigation of floating-point-related issues
* Support of non-trivial 3D scenes

The list of pbrt scenes that I use for regression testing: [some supported pbrt scenes](https://github.com/kennyalive/yar-test-renders/blob/main/pbrt_scenes.list)

The official pbrt 3 scenes are available at: https://www.pbrt.org/scenes-v3
