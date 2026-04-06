if "%VULKAN_SDK%"=="" (
    echo ERROR: VULKAN_SDK is not defined.
    exit /b 1
)
set SLANGC=%VULKAN_SDK%\Bin\slangc.exe
if not exist "%SLANGC%" (
    echo ERROR: slangc not found at "%SLANGC%"
    exit /b 1
)
echo Using slangc: %SLANGC%

set BASEDIR=%~dp0
set SPIRV=%BASEDIR%..\..\data\spirv

%SLANGC% %BASEDIR%\apply_tone_mapping.slang -target spirv -matrix-layout-row-major -o %SPIRV%\apply_tone_mapping.spv
%SLANGC% %BASEDIR%\copy_to_swapchain.slang -target spirv -matrix-layout-row-major -o %SPIRV%\copy_to_swapchain.spv
%SLANGC% %BASEDIR%\patch_materials.slang -target spirv -matrix-layout-row-major -o %SPIRV%\patch_materials.spv
%SLANGC% %BASEDIR%\raytrace_scene.chit.slang -target spirv -matrix-layout-row-major -o %SPIRV%\raytrace_scene.chit.spv
%SLANGC% %BASEDIR%\raytrace_scene.miss.slang -target spirv -matrix-layout-row-major -o %SPIRV%\raytrace_scene.miss.spv
%SLANGC% %BASEDIR%\raytrace_scene.rgen.slang -target spirv -matrix-layout-row-major -o %SPIRV%\raytrace_scene.rgen.spv
%SLANGC% %BASEDIR%\raytrace_scene_shadow_ray.chit.slang -target spirv -matrix-layout-row-major -o %SPIRV%\raytrace_scene_shadow_ray.chit.spv
%SLANGC% %BASEDIR%\path_tracing.chit.slang -target spirv -matrix-layout-row-major -o %SPIRV%\path_tracing.chit.spv
%SLANGC% %BASEDIR%\path_tracing.miss.slang -target spirv -matrix-layout-row-major -o %SPIRV%\path_tracing.miss.spv
%SLANGC% %BASEDIR%\path_tracing.rgen.slang -target spirv -matrix-layout-row-major -o %SPIRV%\path_tracing.rgen.spv
%SLANGC% %BASEDIR%\path_tracing_shadow_ray.chit.slang -target spirv -matrix-layout-row-major -o %SPIRV%\path_tracing_shadow_ray.chit.spv

echo %DATE% %TIME% > %~dp0\..\..\data\spirv\compile_shaders.stamp
