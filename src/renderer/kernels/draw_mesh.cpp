#include "std.h"
#include "lib/common.h"
#include "draw_mesh.h"

#include "renderer/geometry.h"
#include "renderer/kernel_context.h"
#include "renderer/utils.h"
#include "shaders/shared_main.h"

#include "lib/matrix.h"

namespace {
struct Draw_Mesh_Uniform_Buffer {
    Matrix4x4 model_view_proj;
    Matrix4x4 model_view;
    Matrix4x4 view;
    uint32_t point_light_count;
    uint32_t diffuse_rectangular_light_count;
    Vector2 pad0;
};
}

// TODO: temp structure. Use separate buffer per attribute.
struct GPU_Vertex {
    Vector3 position;
    Vector3 normal;
    Vector2 uv;
};

void Draw_Mesh::create(const Kernel_Context& ctx, VkRenderPass render_pass, bool disable_backface_culling, bool front_face_has_clockwise_winding) {
    uniform_buffer = vk_create_mapped_buffer(static_cast<VkDeviceSize>(sizeof(Draw_Mesh_Uniform_Buffer)),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, &mapped_uniform_buffer, "raster_uniform_buffer");

    descriptor_set_layout = Descriptor_Set_Layout()
        .uniform_buffer (0, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT)
        .create         ("raster_set_layout");

    // Pipeline layout.
    {
        VkDescriptorSetLayout set_layouts[] = {
            ctx.base_descriptor_set_layout,
            ctx.material_descriptor_set_layout,
            ctx.light_descriptor_set_layout,
            descriptor_set_layout
        };

        VkPushConstantRange push_constant_range;
        push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
        push_constant_range.offset = 0;
        push_constant_range.size = Compatible_Layout_Push_Constant_Count * sizeof(uint32_t);

        VkPipelineLayoutCreateInfo create_info{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        create_info.setLayoutCount = (uint32_t)std::size(set_layouts);
        create_info.pSetLayouts = set_layouts;
        create_info.pushConstantRangeCount = 1;
        create_info.pPushConstantRanges = &push_constant_range;

        VK_CHECK(vkCreatePipelineLayout(vk.device, &create_info, nullptr, &pipeline_layout));
        vk_set_debug_name(pipeline_layout, "draw_mesh_pipeline_layout");
    }

    // Pipeline.
    {
        Shader_Module vertex_shader("spirv/draw_mesh.vert.spv");
        Shader_Module fragment_shader("spirv/draw_mesh.frag.spv");

        Vk_Graphics_Pipeline_State state = get_default_graphics_pipeline_state();

        // VkVertexInputBindingDescription
        state.vertex_bindings[0].binding = 0;
        state.vertex_bindings[0].stride = sizeof(GPU_Vertex);
        state.vertex_bindings[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        state.vertex_binding_count = 1;

        // VkVertexInputAttributeDescription
        state.vertex_attributes[0].location = 0; // vertex
        state.vertex_attributes[0].binding = 0;
        state.vertex_attributes[0].format = VK_FORMAT_R32G32B32_SFLOAT;
        state.vertex_attributes[0].offset = 0;

        state.vertex_attributes[1].location = 1; // normal
        state.vertex_attributes[1].binding = 0;
        state.vertex_attributes[1].format = VK_FORMAT_R32G32B32_SFLOAT;
        state.vertex_attributes[1].offset = 12;

        state.vertex_attributes[2].location = 2; // uv
        state.vertex_attributes[2].binding = 0;
        state.vertex_attributes[2].format = VK_FORMAT_R32G32_SFLOAT;
        state.vertex_attributes[2].offset = 24;
        state.vertex_attribute_count = 3;

        if (disable_backface_culling)
            state.rasterization_state.cullMode = VK_CULL_MODE_NONE;
        else if (front_face_has_clockwise_winding)
            state.rasterization_state.frontFace = VK_FRONT_FACE_CLOCKWISE;

        pipeline = vk_create_graphics_pipeline(state, pipeline_layout, render_pass, vertex_shader.handle, fragment_shader.handle);
    }

    //
    // Descriptor sets.
    //
    {
        VkDescriptorSetAllocateInfo desc { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        desc.descriptorPool     = vk.descriptor_pool;
        desc.descriptorSetCount = 1;
        desc.pSetLayouts        = &descriptor_set_layout;
        VK_CHECK(vkAllocateDescriptorSets(vk.device, &desc, &descriptor_set));

        Descriptor_Writes(descriptor_set)
            .uniform_buffer (0, uniform_buffer.handle, 0, sizeof(Draw_Mesh_Uniform_Buffer));
    }
}

void Draw_Mesh::destroy() {
    uniform_buffer.destroy();
    vkDestroyDescriptorSetLayout(vk.device, descriptor_set_layout, nullptr);
    vkDestroyPipelineLayout(vk.device, pipeline_layout, nullptr);
    vkDestroyPipeline(vk.device, pipeline, nullptr);
    *this = Draw_Mesh{};
}

void Draw_Mesh::update_point_lights(int light_count) {
    Draw_Mesh_Uniform_Buffer& buf = *static_cast<Draw_Mesh_Uniform_Buffer*>(mapped_uniform_buffer);
    buf.point_light_count = light_count;
}

void Draw_Mesh::update_diffuse_rectangular_lights(int light_count) {
    Draw_Mesh_Uniform_Buffer& buf = *static_cast<Draw_Mesh_Uniform_Buffer*>(mapped_uniform_buffer);
    buf.diffuse_rectangular_light_count = light_count;
}

void Draw_Mesh::update(const Matrix3x4& view_transform, float fov) {
    float aspect_ratio = (float)vk.surface_size.width / (float)vk.surface_size.height;
    Matrix3x4 from_world_to_opengl = {{
        {1,  0, 0, 0},
        {0,  0, 1, 0},
        {0, -1, 0, 0}
    }};
    Matrix4x4 proj = perspective_transform_opengl_z01(radians(fov), aspect_ratio, 0.1f, 5000.0f) * from_world_to_opengl;
    Matrix4x4 model_view = Matrix4x4::identity * view_transform;
    Matrix4x4 model_view_proj = proj * view_transform;

    Draw_Mesh_Uniform_Buffer& buf = *static_cast<Draw_Mesh_Uniform_Buffer*>(mapped_uniform_buffer);
    buf.model_view_proj = model_view_proj;
    buf.model_view = model_view;
    buf.view = Matrix4x4::identity * view_transform;
}

void Draw_Mesh::bind_sets_and_pipeline() {
    vkCmdBindDescriptorSets(vk.command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout, KERNEL_SET_0, 1, &descriptor_set, 0, nullptr);
    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
}

void Draw_Mesh::dispatch(const GPU_Mesh& gpu_mesh, int instance_index) {
    const VkDeviceSize zero_offset = 0;
    vkCmdBindVertexBuffers(vk.command_buffer, 0, 1, &gpu_mesh.vertex_buffer.handle, &zero_offset);
    vkCmdBindIndexBuffer(vk.command_buffer, gpu_mesh.index_buffer.handle, 0, VK_INDEX_TYPE_UINT32);
    vkCmdPushConstants(vk.command_buffer, pipeline_layout, VK_SHADER_STAGE_ALL, 0, sizeof(int32_t),  &instance_index);
    vkCmdDrawIndexed(vk.command_buffer, gpu_mesh.model_index_count, 1, 0, 0, 0);
}

